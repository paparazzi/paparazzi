import wx

class TextDropTarget(wx.TextDropTarget):
    """ This object implements Drop Target functionality for Text """
    def __init__(self, reference):
        """ Initialize the Drop Target, passing in the Object Reference to
        indicate what should receive the dropped text """
        wx.TextDropTarget.__init__(self)
        self.reference = reference

    def OnDropText(self, x, y, data):
        """ When text is dropped, send it to the object specified """
        self.reference.OnDropText(data)
