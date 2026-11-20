"""Display measured XYZ surfels and meshes posed by archived MuJoCo states.

The display footprints and PCA normals do not replace physics geometry.
"""

import os

os.environ.setdefault("VTK_DEFAULT_OPENGL_WINDOW", "vtkEGLRenderWindow")
os.environ.setdefault("VTK_SMP_MAX_THREADS", "4")

import mujoco as mj
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

PALETTE = ["#a6afb6", "#4fb9cf", "#2c9c9a", "#dfb554", "#cb6548"]


class MeasuredRenderer:
    def __init__(self, model, cloud, poses, scene, size=(1740, 870)):
        self.model, self.scene, self.size = model, scene, size
        center = np.median(poses[:, :2], axis=0)
        radius = 17 + np.max(np.linalg.norm(poses[:, :2] - center, axis=1))
        local = cloud[np.linalg.norm(cloud[:, :2] - center, axis=1) < radius]
        self.floor = float(np.percentile(local[:, 2], 8))
        self.height_span = 2.0
        local = local[
            (local[:, 2] > self.floor - 0.3)
            & (local[:, 2] < self.floor + (2.0 if scene == "indoor" else 3.5))
        ]
        points = vtk.vtkPoints()
        points.SetData(numpy_to_vtk(np.ascontiguousarray(local), deep=True))
        poly = vtk.vtkPolyData()
        poly.SetPoints(points)
        levels = numpy_to_vtk(np.ascontiguousarray(local[:, 2]), deep=True)
        levels.SetName("height")
        poly.GetPointData().SetScalars(levels)
        locator = vtk.vtkStaticPointLocator()
        locator.SetDataSet(poly)
        locator.BuildLocator()
        nearest = vtk.vtkIdList()
        radii = np.empty(len(local), dtype=np.float32)
        for i, point in enumerate(local):
            locator.FindClosestNPoints(5, point, nearest)
            neighbor = local[nearest.GetId(nearest.GetNumberOfIds() - 1)]
            radii[i] = np.clip(0.7 * np.linalg.norm(neighbor - point), 0.045, 0.095)
        radius_array = numpy_to_vtk(radii, deep=True)
        radius_array.SetName("display_radius")
        poly.GetPointData().AddArray(radius_array)
        normals = vtk.vtkPCANormalEstimation()
        normals.SetInputData(poly)
        normals.SetSampleSize(16)
        normals.SetNormalOrientationToGraphTraversal()
        normals.Update()
        normal_poly = normals.GetOutput()
        normal_poly.GetPointData().GetNormals().SetName("Normals")
        disk = vtk.vtkDiskSource()
        disk.SetOuterRadius(1.0)
        disk.SetInnerRadius(0)
        disk.SetCircumferentialResolution(12)
        disk.SetRadialResolution(1)
        transform = vtk.vtkTransform()
        transform.RotateY(90)  # Glyph3D's direction axis is X; disks face Z.
        transformed = vtk.vtkTransformFilter()
        transformed.SetTransform(transform)
        transformed.SetInputConnection(disk.GetOutputPort())
        mapper = vtk.vtkGlyph3DMapper()
        mapper.SetInputData(normal_poly)
        mapper.SetSourceConnection(transformed.GetOutputPort())
        mapper.SetScaleArray("display_radius")
        mapper.SetScaleModeToScaleByMagnitude()
        mapper.SetScaleFactor(1.0)
        mapper.ScalingOn()
        mapper.SetOrientationArray("Normals")
        mapper.SetOrientationModeToDirection()
        mapper.OrientOn()
        lookup = vtk.vtkColorTransferFunction()
        for height, color in zip(
            np.linspace(self.floor, self.floor + self.height_span, 5), PALETTE
        ):
            lookup.AddRGBPoint(
                height, *[int(color[k : k + 2], 16) / 255 for k in (1, 3, 5)]
            )
        mapper.SetLookupTable(lookup)
        mapper.SetScalarRange(self.floor, self.floor + self.height_span)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetAmbient(0.65)
        actor.GetProperty().SetDiffuse(0.35)
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.835, 0.862, 0.878)
        self.renderer.AddActor(actor)
        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.SetSize(*size)
        self.window.SetMultiSamples(0)
        self.window.AddRenderer(self.renderer)
        basic = vtk.vtkRenderStepsPass()
        edl = vtk.vtkEDLShading()
        edl.SetDelegatePass(basic)
        self.edl = edl
        self.renderer.SetPass(edl)
        self.meshes = []
        for geom in range(model.ngeom):
            if model.geom_bodyid[geom] == 0 or model.geom_group[geom] == 3:
                continue
            if model.geom_type[geom] != mj.mjtGeom.mjGEOM_MESH:
                continue
            mesh = model.geom_dataid[geom]
            first, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[first : first + count].copy()
            first, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            faces = model.mesh_face[first : first + count].copy()
            vp = vtk.vtkPoints()
            vp.SetData(numpy_to_vtk(vertices, deep=True))
            cells = vtk.vtkCellArray()
            packed = np.c_[np.full(count, 3), faces].astype(np.int64).ravel()
            cells.ImportLegacyFormat(numpy_to_vtkIdTypeArray(packed, deep=True))
            shape = vtk.vtkPolyData()
            shape.SetPoints(vp)
            shape.SetPolys(cells)
            mesh_normals = vtk.vtkPolyDataNormals()
            mesh_normals.SetInputData(shape)
            mesh_normals.SetFeatureAngle(60)
            mesh_mapper = vtk.vtkPolyDataMapper()
            mesh_mapper.SetInputConnection(mesh_normals.GetOutputPort())
            mesh_actor = vtk.vtkActor()
            mesh_actor.SetMapper(mesh_mapper)
            mesh_actor.GetProperty().SetColor(0.75, 0.78, 0.82)
            mesh_actor.GetProperty().SetSpecular(0.35)
            mesh_actor.GetProperty().SetSpecularPower(25)
            matrix = vtk.vtkMatrix4x4()
            mesh_actor.SetUserMatrix(matrix)
            self.renderer.AddActor(mesh_actor)
            self.meshes.append((geom, matrix))
        self.shadows = []
        for radius, opacity in [
            (0.45, 0.025),
            (0.36, 0.035),
            (0.27, 0.045),
            (0.18, 0.055),
        ]:
            disk = vtk.vtkDiskSource()
            disk.SetOuterRadius(radius)
            disk.SetInnerRadius(0)
            disk.SetCircumferentialResolution(40)
            shadow_mapper = vtk.vtkPolyDataMapper()
            shadow_mapper.SetInputConnection(disk.GetOutputPort())
            shadow = vtk.vtkActor()
            shadow.SetMapper(shadow_mapper)
            shadow.SetScale(1, 0.55, 1)
            shadow.GetProperty().SetColor(0.08, 0.10, 0.12)
            shadow.GetProperty().SetOpacity(opacity)
            shadow.GetProperty().LightingOff()
            self.renderer.AddActor(shadow)
            self.shadows.append(shadow)
        self.trace_points = vtk.vtkPoints()
        self.trace_poly = vtk.vtkPolyData()
        self.trace_poly.SetPoints(self.trace_points)
        tube = vtk.vtkTubeFilter()
        tube.SetInputData(self.trace_poly)
        tube.SetRadius(0.025)
        tube.SetNumberOfSides(8)
        trace_mapper = vtk.vtkPolyDataMapper()
        trace_mapper.SetInputConnection(tube.GetOutputPort())
        trace_actor = vtk.vtkActor()
        trace_actor.SetMapper(trace_mapper)
        trace_actor.GetProperty().SetColor(0.33, 0.84, 0.73)
        trace_actor.GetProperty().LightingOff()
        self.renderer.AddActor(trace_actor)
        self.capture = vtk.vtkWindowToImageFilter()
        self.capture.SetInput(self.window)
        self.capture.ReadFrontBufferOff()
        self.point_count = len(local)

    def render(self, data, trace):
        for shadow in self.shadows:
            shadow.SetPosition(data.qpos[0], data.qpos[1], float(trace[-1, 2] - 0.025))
        for geom, matrix in self.meshes:
            rotation = data.geom_xmat[geom].reshape(3, 3)
            for row in range(3):
                for col in range(3):
                    matrix.SetElement(row, col, rotation[row, col])
                matrix.SetElement(row, 3, data.geom_xpos[geom, row])
        self.trace_points.SetData(numpy_to_vtk(np.ascontiguousarray(trace), deep=True))
        line = vtk.vtkCellArray()
        line.InsertNextCell(len(trace))
        for i in range(len(trace)):
            line.InsertCellPoint(i)
        self.trace_poly.SetLines(line)
        self.trace_poly.Modified()
        camera = self.renderer.GetActiveCamera()
        camera.SetFocalPoint(*(data.qpos[:3] + [0, 0, 0.1]))
        camera.SetPosition(*(data.qpos[:3] + np.array([-0.7, -0.8, 0.6]) * 4.5))
        camera.SetViewUp(0, 0, 1)
        camera.SetClippingRange(0.03, 1000)
        self.renderer.SetPass(self.edl)
        self.window.Render()
        self.capture.Modified()
        self.capture.Update()
        width, height = self.size
        shaded = (
            vtk_to_numpy(self.capture.GetOutput().GetPointData().GetScalars())
            .reshape(height, width, 3)[::-1]
            .copy()
        )
        self.renderer.SetPass(None)
        self.window.Render()
        self.capture.Modified()
        self.capture.Update()
        base = (
            vtk_to_numpy(self.capture.GetOutput().GetPointData().GetScalars())
            .reshape(height, width, 3)[::-1]
            .copy()
        )
        return np.rint(0.15 * shaded + 0.85 * base).astype(np.uint8)

    def close(self):
        self.edl.ReleaseGraphicsResources(self.window)
        self.window.Finalize()
