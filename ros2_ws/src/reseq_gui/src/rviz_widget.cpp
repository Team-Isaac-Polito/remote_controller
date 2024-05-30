#include "reseq_gui/rviz_widget.hpp"

RVizWidget::RVizWidget(QApplication* app) : app_(app) {
    using RosNode = rviz_common::ros_integration::RosNodeAbstraction;
    // Create the main RViz widget
    widget_ = new QWidget();
    rviz_ros_node_ = std::make_shared<RosNode>("reseq_rviz_widget");

    initializeRViz();
    loadViews();
    initDisplays();
}

void RVizWidget::pause() { manager_->stopUpdate(); }

void RVizWidget::resume() { manager_->startUpdate(); }

void RVizWidget::initializeRViz() {
    app_->processEvents();
    render_panel_ = new rviz_common::RenderPanel(widget_);
    app_->processEvents();
    render_panel_->getRenderWindow()->initialize();

    auto clock = rviz_ros_node_.lock()->get_raw_node()->get_clock();
    rviz_common::WindowManagerInterface* wm_ = nullptr;
    manager_ = new rviz_common::VisualizationManager(
        render_panel_, rviz_ros_node_, wm_, clock);

    manager_->setFixedFrame("base_link");

    view_manager_ = manager_->getViewManager();
    render_panel_->initialize(manager_);
    app_->processEvents();
    manager_->initialize();
}

void RVizWidget::loadViews() {
    rviz_common::ViewController* view;

    view = view_manager_->create("rviz_default_plugins/Orbit");
    view->setName("Third Person");
    view->subProp("Distance")->setValue(3);
    view->subProp("Pitch")->setValue(0.3);
    view->subProp("Yaw")->setValue(3.1415);

    view_manager_->add(view, 0);
    view_manager_->setCurrentFrom(view);

    view = view_manager_->create("rviz_default_plugins/TopDownOrtho");
    view->setName("Top-Down");
    view->subProp("Scale")->setValue(125);
    view->subProp("Angle")->setValue(-1.57);

    view_manager_->add(view, 1);

    view = view_manager_->create("rviz_default_plugins/FPS");
    view->setName("First Person");
    view->subProp("Position")->subProp("Z")->setValue(0.5);

    view_manager_->add(view, 2);

    view = view_manager_->create("rviz_default_plugins/Orbit");
    view->setName("Third Person (front)");
    view->subProp("Distance")->setValue(3);
    view->subProp("Pitch")->setValue(0.3);
    view->subProp("Yaw")->setValue(0);

    view_manager_->add(view, 3);

    view = view_manager_->create("rviz_default_plugins/Orbit");
    view->setName("Isometric (left)");
    view->subProp("Distance")->setValue(4);
    view->subProp("Pitch")->setValue(0.3);
    view->subProp("Yaw")->setValue(2.356);

    view_manager_->add(view, 4);

    view = view_manager_->create("rviz_default_plugins/Orbit");
    view->setName("Isometric (right)");
    view->subProp("Distance")->setValue(4);
    view->subProp("Pitch")->setValue(0.3);
    view->subProp("Yaw")->setValue(3.927);

    view_manager_->add(view, 5);

    view = view_manager_->create("rviz_default_plugins/Orbit");
    view->setName("Side (left)");
    view->subProp("Distance")->setValue(4);
    view->subProp("Pitch")->setValue(0.3);
    view->subProp("Yaw")->setValue(1.57);

    view_manager_->add(view, 6);

    view = view_manager_->create("rviz_default_plugins/Orbit");
    view->setName("Side (right)");
    view->subProp("Distance")->setValue(4);
    view->subProp("Pitch")->setValue(0.3);
    view->subProp("Yaw")->setValue(4.712);

    view_manager_->add(view, 7);
}
}

void RVizWidget::displayGrid(bool grid) { grid_->setEnabled(grid); }

void RVizWidget::displayRobotModel(bool robot) {
    robot_model_->setEnabled(robot);
}

void RVizWidget::initDisplays() {
    grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "grid", true);

    robot_model_ = manager_->createDisplay(
        "rviz_default_plugins/RobotModel", "RESEQ", true);
    robot_model_->subProp("Description Topic")->setValue("/robot_description");
}
void RVizWidget::setView(RVizView view) {
    int index = static_cast<int>(view);
    view_manager_->setCurrentFrom(view_manager_->getViewAt(index));
}
