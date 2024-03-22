from src.plugin_interface import  PluginInterface
from PyQt6.QtWidgets import QWidget
from .stereo_main import Ui_MainWindow
import cv2


class Controller(QWidget):
    def __init__(self, model):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.model = model
        self.image = None
        self.set_styleSheet()


    def set_styleSheet(self):
        # Titles
        self.ui.title.setStyleSheet(self.model.style_label_title())
        self.ui.credit.setStyleSheet(self.model.style_label_title())
        self.ui.cam0.setStyleSheet(self.model.style_label_title())
        self.ui.cam1.setStyleSheet(self.model.style_label_title())
        self.ui.mapViewer.setStyleSheet(self.model.style_label_title())
        self.ui.camCons.setStyleSheet(self.model.style_label_title())
        self.ui.mapping.setStyleSheet(self.model.style_label_title())
        self.ui.staging.setStyleSheet(self.model.style_label_title())
        self.ui.other.setStyleSheet(self.model.style_label_title())

        # Buttons
        self.ui.loadData.setStyleSheet(self.model.style_pushbutton())
        self.ui.param.setStyleSheet(self.model.style_pushbutton())
        self.ui.camView.setStyleSheet(self.model.style_pushbutton())
        self.ui.topView.setStyleSheet(self.model.style_pushbutton())
        self.ui.reset.setStyleSheet(self.model.style_pushbutton())
        self.ui.stop.setStyleSheet(self.model.style_pushbutton())
        self.ui.stepButton.setStyleSheet(self.model.style_pushbutton())

        # Checkboxes
        self.ui.followCam.setStyleSheet(self.model.style_checkbox())
        self.ui.showPoints.setStyleSheet(self.model.style_checkbox())
        self.ui.showKey.setStyleSheet(self.model.style_checkbox())
        self.ui.showGraph.setStyleSheet(self.model.style_checkbox())
        self.ui.showInert.setStyleSheet(self.model.style_checkbox())
        self.ui.local.setStyleSheet(self.model.style_checkbox())
        self.ui.stepCheck.setStyleSheet(self.model.style_checkbox())
        self.ui.showLBA.setStyleSheet(self.model.style_checkbox())

        # Frames
        self.ui.cam0Screen.setStyleSheet(self.model.style_label())
        self.ui.cam1Screen.setStyleSheet(self.model.style_label())
        self.ui.mapScreen.setStyleSheet(self.model.style_label())

        # Line
        self.ui.line.setStyleSheet(self.model.style_line())
        self.ui.line_2.setStyleSheet(self.model.style_line())
        self.ui.line_3.setStyleSheet(self.model.style_line())
        self.ui.line_4.setStyleSheet(self.model.style_line())
        self.ui.line_5.setStyleSheet(self.model.style_line())

        # This is set up to connect to other function to make action on ui
        self.ui.loadData.clicked.connect(self.load_image)
        self.ui.param.clicked.connect(self.cam_params)


    def load_image(self):
        file = self.model.select_file()
        if file:
            self.moildev = self.model.connect_to_moildev(parameter_name=file)
        self.image_original = cv2.imread(file)
        self.image = self.image_original.copy()
        self.show_to_ui()

    def show_to_ui(self):
        # map_x, map_y = self.moildev.maps_anypoint_mode1(0, 0, 4)
        # draw_image = self.model.draw_polygon(self.image, map_x, map_y)

        self.model.show_image_to_label(self.ui.cam0Screen, self.image, 400)
        self.model.show_image_to_label(self.ui.cam1Screen, self.image, 400)

    def cam_params(self):
        self.model.form_camera_parameter()


class Amcl3dOrbSlam3(PluginInterface):
    def __init__(self):
        super().__init__()
        self.widget = None
        self.description = "This is a plugins application for mono/stereo ORB SLAM3 visualization"

    def set_plugin_widget(self, model):
        self.widget = Controller(model)
        return self.widget

    def set_icon_apps(self):
        return "icon.png"

    def change_stylesheet(self):
        self.widget.set_stylesheet()
