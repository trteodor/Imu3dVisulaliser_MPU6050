#ifndef GENERICQCP_H
#define GENERICQCP_H


#include <QMainWindow>

#include "QWidget"
#include "qcustomplot.h"

class GenericQCP : public QMainWindow
{
    Q_OBJECT
public:
    explicit GenericQCP(void);
    ~GenericQCP();

    void LfGraphInitialize(QCustomPlot *UIPassedplot,QCPGraph::LineStyle LineStyle);
    void Graph_AppendData(float X_Pos1,float Y_Pos1,
                            float X_Pos2=0,float Y_Pos2=0,
                            float X_Pos3=0,float Y_Pos3=0,
                            float X_Pos4=0,float Y_Pos4=0,
                            float X_Pos5=0,float Y_Pos5=0,
                            float X_Pos6=0,float Y_Pos6=0);
    void Graph_UpdateReplot(void);
    void Graph_ClearData(void);
    void Graph_DrawMarkersAtDataIndex(int DataIndex);


    QCustomPlot *UIplotP;

    QVector<double> DataVector_X1, DataVector_Y1;
    QVector<double> DataVector_X2, DataVector_Y2;
    QVector<double> DataVector_X3, DataVector_Y3;
    QVector<double> DataVector_X4, DataVector_Y4;
    QVector<double> DataVector_X5, DataVector_Y5;
    QVector<double> DataVector_X6, DataVector_Y6;



signals:
    void LfGraphSignal_graphClicked(int dataIndex);

private slots:
    /******************************************************************************/

    void Graph_axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
    void Graph_legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
    void Graph_selectionChanged();
    void Graph_mousePress();
    void Graph_mouseWheel();
    void Graph_addRandomGraph();
    void Graph_removeSelectedGraph();
    void Graph_removeAllGraphs();
    void Graph_contextMenuRequest(QPoint pos);
    void Graph_moveLegend();
    void Graph_graphClicked(QCPAbstractPlottable *plottable, int dataIndex);
    /******************************************************************************/

private:


    QPointer<QCPGraph> Graph1;
    QPointer<QCPGraph> Graph2;
    QPointer<QCPGraph> Graph3;
    QPointer<QCPGraph> Graph4;
    QPointer<QCPGraph> Graph5;
    QPointer<QCPGraph> Graph6;



    QVector<double> DataVector_PointX_x, DataVector_PointX_y;
    QVector<double> DataVector_PointY_x, DataVector_PointY_y;


    QPointer<QCPGraph> SelectedPointMarkerGraphY;
    QPointer<QCPGraph> SelectedPointMarkerGraphX;





};





#endif // GENERICQCP_H
