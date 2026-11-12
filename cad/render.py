"""Offscreen VTK renders of assemblies / parts (software GL, no display needed)."""
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy
from PIL import Image

COLORS = {"PLA": (0.62, 0.72, 0.85), "PETG": (0.95, 0.55, 0.15), "TPU": (0.22, 0.22, 0.25),
          "servo": (0.25, 0.25, 0.28), "bom": (0.25, 0.65, 0.35), "hw": (0.75, 0.75, 0.78)}


def _actor(shape, color, opacity=1.0, tol=0.15):
    pd = shape.toVtkPolyData(tol, 0.3)
    nrm = vtk.vtkPolyDataNormals(); nrm.SetInputData(pd); nrm.SetFeatureAngle(40); nrm.SplittingOn(); nrm.Update()
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nrm.GetOutputPort())
    a = vtk.vtkActor(); a.SetMapper(m)
    p = a.GetProperty(); p.SetColor(*color); p.SetOpacity(opacity); p.SetSpecular(0.25); p.SetSpecularPower(20)
    return a


def render(items, path=None, view=(1, -1, 0.8), size=(900, 700), bounds=None, up=(0, 0, 1), zoom=1.0,
           top_hidden=(), opacity=None, focal=None, dist_scale=1.0):
    """items: [(name, kind, shape, leg)] -> PIL image (also saved when path given)"""
    ren = vtk.vtkRenderer(); ren.SetBackground(1, 1, 1)
    win = vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.AddRenderer(ren); win.SetSize(*size)
    win.SetMultiSamples(4)
    for name, kind, shape, leg in items:
        if name in top_hidden:
            continue
        op = 1.0 if opacity is None else opacity.get(kind, 1.0)
        ren.AddActor(_actor(shape, COLORS.get(kind, (0.6, 0.6, 0.6)), op))
    ren.ResetCamera()
    b = ren.ComputeVisiblePropBounds() if bounds is None else bounds
    c = focal or ((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2)
    diag = max(b[1] - b[0], b[3] - b[2], b[5] - b[4])
    v = np.array(view, float); v /= np.linalg.norm(v)
    cam = ren.GetActiveCamera()
    cam.SetFocalPoint(*c); cam.SetPosition(*(np.array(c) + v * diag * 2.2 * dist_scale)); cam.SetViewUp(*up)
    cam.SetViewAngle(25); cam.Zoom(zoom)
    ren.ResetCameraClippingRange()
    light = vtk.vtkLight(); light.SetLightTypeToHeadlight(); ren.AddLight(light)
    win.Render()
    f = vtk.vtkWindowToImageFilter(); f.SetInput(win); f.ReadFrontBufferOff(); f.Update()
    img = f.GetOutput(); w, h, _ = img.GetDimensions()
    arr = vtk_to_numpy(img.GetPointData().GetScalars()).reshape(h, w, -1)[::-1, :, :3]
    im = Image.fromarray(arr.astype(np.uint8))
    if path:
        im.save(path)
    return im
