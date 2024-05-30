#pragma once

#include <QApplication>
#include <QWidget>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "reseq_gui/reseq_widget.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"

namespace rviz_common {
class Display;
class RenderPanel;
class VisualizationManager;
class ViewManager;
class ViewController;
}  // namespace rviz_common

class RVizWidget : public ReseQWidget {
   public:
    enum class RVizView {
        THIRD_PERSON = 0,
        TOP_DOWN,
        FIRST_PERSON,
        THIRD_PERSON_FRONT,
        ISOMETRIC_LEFT,
        ISOMETRIC_RIGHT,
        SIDE_LEFT,
        SIDE_RIGHT,
    };

    /**
     * @brief Creates the main RViz widget with the visualizations.
     *
     * @details At creation the widget creates `Display` for the grid and the
     * robot model. The widget is stopped
     *
     * @param app The Qt application class
     */
    RVizWidget(QApplication* app);

    /**
     * @brief Adds or remove the grid display
     * @param grid If true, the grid display is added, otherwise it is removed
     * (default true)
     */
    void displayGrid(bool grid = true);

    /**
     * @brief Adds or remove the robot model display
     * @param robot If true, the robot model display is added, otherwise it is
     * removed (default true)
     */
    void displayRobotModel(bool robot = true);

    /**
     * @brief Changes the View of the RViz widget
     * @param view The view to change to
     */
    void setView(RVizView view);

    void resume() override;
    void pause() override;
    void actionCallback(const int button_id) override;
    QWidget* getWidget() override;
    QStringList* getButtonLabels() override;


   private:
    void initializeRViz();
    void loadViews();
    void initDisplays();

    QApplication* app_;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
        rviz_ros_node_;

    rviz_common::RenderPanel* render_panel_;
    rviz_common::Display* grid_;
    rviz_common::Display* robot_model_;
    rviz_common::ViewManager* view_manager_;
    rviz_common::VisualizationManager* manager_;
};