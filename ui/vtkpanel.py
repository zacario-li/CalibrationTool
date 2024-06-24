import vtk
import wx
from vtk.wx.wxVTKRenderWindowInteractor import wxVTKRenderWindowInteractor
from vtk.wx.wxVTKRenderWindow import wxVTKRenderWindow


class VTKPanel(wx.Panel):
    def __init__(self, parent, size, src=None):
        wx.Panel.__init__(self, parent, id=wx.ID_ANY, size=size)
        self.widget = wxVTKRenderWindowInteractor(self, -1)
        self.widget.Enable(1)
        self.src = src

        self.iren = self.widget.GetRenderWindow().GetInteractor()
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(style)

        self.ren = vtk.vtkRenderer()
        self.widget.GetRenderWindow().AddRenderer(self.ren)

        axes = vtk.vtkAxesActor()
        # 务必使用self来保持marker的完整生命周期，否则会无法在窗口显示
        self.marker = vtk.vtkOrientationMarkerWidget()
        self.marker.SetInteractor(self.widget._Iren)
        self.marker.SetOrientationMarker(axes)
        self.marker.SetEnabled(1)
        self.marker.InteractiveOff()

        # camera widget
        self.marker1 = vtk.vtkCameraOrientationWidget()
        self.marker1.SetParentRenderer(self.ren)
        self.marker1.On()

    def render(self, src=None):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(self.src if src is None else src)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        self.ren.AddActor(actor)

        self.ren.ResetCamera()
        self.Refresh()
