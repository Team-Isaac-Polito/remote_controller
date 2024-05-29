#pragma once

#include <QStringList>
#include <QWidget>


class ReseQWidget {
   public:
    /**
     * @brief Get the widget of the ReseQWidget
     * @return QWidget* The widget of the ReseQWidget
     */
    virtual QWidget* getWidget() = 0;

    /**
     * @brief Get the GUI buttons labels of the ReseQWidget 
     * @return `QStringList*` The button labels of the ReseQWidget
     */
    virtual QStringList* getButtonLabels() = 0;

    /**
     * @brief Callback called by the main window when a button is pressed
     * @param button_id The id of the button pressed
    */
    virtual void actionCallback(const int button_id) = 0;

    /**
     * @brief Callback called whem the widget is added to
     * the main window
    */
    virtual void resume() = 0;

    /**
     * @brief Callback called whem the widget is removed from
     * the main window
    */
    virtual void pause() = 0;

   protected:
    QWidget* widget_;
    QStringList* context_labels_;
};