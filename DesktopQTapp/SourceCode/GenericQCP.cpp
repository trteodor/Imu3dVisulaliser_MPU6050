#include "GenericQCP.h"


GenericQCP::GenericQCP(void)
{
//    qDebug() << "Construct Called Class:GenericQCP";
}


void GenericQCP::LfGraphInitialize(QCustomPlot *UIPassedplot,QCPGraph::LineStyle LineStyle)
{
    UIplotP = UIPassedplot;

    //    ui->MapViewWidget
    UIplotP->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                             QCP::iSelectLegend | QCP::iSelectPlottables);
    UIplotP->xAxis->setRange(-8, 8);
    UIplotP->yAxis->setRange(-3, 3);
    UIplotP->axisRect()->setupFullAxesBox();
    //    UIplotP->plotLayout()->insertRow(0);
    //    QCPTextElement *title = new QCPTextElement(ui->MapViewWidget, "Interaction Example", QFont("sans", 17, QFont::Bold));
    //    UIplotP->plotLayout()->addElement(0, 0, title);
    UIplotP->xAxis->setLabel("x Axis");
    UIplotP->yAxis->setLabel("y Axis");
    UIplotP->legend->setVisible(false);
    //TODO:
    QFont legendFont = font();
    legendFont.setPointSize(10);
    UIplotP->legend->setFont(legendFont);
    UIplotP->legend->setSelectedFont(legendFont);
    UIplotP->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items

    UIplotP->rescaleAxes();
    // connect slot that ties some axis selections together (especially opposite axes):
    connect(UIplotP, SIGNAL(selectionChangedByUser()), this, SLOT(Graph_selectionChanged()));
    // connect slots that takes care that when an axis is selected, only that direction can be dragged and zoomed:
    connect(UIplotP, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(Graph_mousePress()));
    connect(UIplotP, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(Graph_mouseWheel()));
    // make bottom and left axes transfer their ranges to top and right axes:
    connect(UIplotP->xAxis, SIGNAL(rangeChanged(QCPRange)), UIplotP->xAxis2, SLOT(setRange(QCPRange)));
    connect(UIplotP->yAxis, SIGNAL(rangeChanged(QCPRange)), UIplotP->yAxis2, SLOT(setRange(QCPRange)));
    // connect some interaction slots:
    connect(UIplotP, SIGNAL(axisDoubleClick(QCPAxis*,QCPAxis::SelectablePart,QMouseEvent*)), this, SLOT(Graph_axisLabelDoubleClick(QCPAxis*,QCPAxis::SelectablePart)));
    connect(UIplotP, SIGNAL(legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*,QMouseEvent*)), this, SLOT(Graph_legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*)));
    //    connect(title, SIGNAL(doubleClicked(QMouseEvent*)), this, SLOT(Graph_titleDoubleClick(QMouseEvent*)));
    // connect slot that shows a message in the status bar when a graph is clicked:
    connect(UIplotP, SIGNAL(plottableClick(QCPAbstractPlottable*,int,QMouseEvent*)), this, SLOT(Graph_graphClicked(QCPAbstractPlottable*,int)));

    // setup policy and connect slot for context menu popup:
    UIplotP->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(UIplotP, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(Graph_contextMenuRequest(QPoint)));

    Graph1 = UIplotP->addGraph();

    Graph1->setLineStyle(LineStyle);
    if(LineStyle == QCPGraph::LineStyle::lsNone)
    {
        Graph1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::black, 1.5), QBrush(Qt::white), 5));
    }
    else
    {
        Graph1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, QPen(Qt::black, 1.5), QBrush(Qt::white), 3));
    }
    Graph1->setPen(QPen(QColor(180, 180, 180), 2));

        QPen redDotPen;
        redDotPen.setStyle(Qt::DotLine);
        redDotPen.setColor(QColor(255, 255, 255, 180));
        redDotPen.setWidthF(1.5);
/***********************************/
        Graph2 = UIplotP->addGraph();
        redDotPen.setColor(QColor(166, 3, 3, 180));
        Graph2->setPen(redDotPen);
        Graph2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, QPen(Qt::red, 1.5), QBrush(Qt::red), 3));

/***********************************/
        Graph3 = UIplotP->addGraph();
        redDotPen.setColor(QColor(255, 255, 255, 180));
        Graph3->setPen(redDotPen);
        Graph3->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, QPen(Qt::blue, 1.5), QBrush(Qt::blue), 3));
/***********************************/
        Graph4 = UIplotP->addGraph();
        Graph4->setPen(redDotPen);
        Graph4->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, QPen(Qt::green, 1.5), QBrush(Qt::green), 3));
/***********************************/
        Graph5 = UIplotP->addGraph();
        Graph5->setPen(redDotPen);
        Graph5->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, QPen(Qt::white, 1.5), QBrush(Qt::white), 3));
/***********************************/
        Graph6 = UIplotP->addGraph();
        Graph6->setPen(redDotPen);
        Graph6->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, QPen(Qt::white, 1.5), QBrush(Qt::white), 3));
/***********************************/
        SelectedPointMarkerGraphX = UIplotP->addGraph();
        redDotPen.setColor(QColor(2, 156, 161, 180));
        SelectedPointMarkerGraphX->setPen(redDotPen);
        SelectedPointMarkerGraphX->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, QPen(Qt::white, 1.5), QBrush(Qt::white), 3));
        /***********************************/
        SelectedPointMarkerGraphY = UIplotP->addGraph();
        SelectedPointMarkerGraphY->setPen(redDotPen);
        SelectedPointMarkerGraphY->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, QPen(Qt::white, 1.5), QBrush(Qt::white), 3));
/***********************************/

//        //Graph2->set
//        QVector<double>  Test_X;
//        QVector<double>  Test_Y;
//        Test_X.append( ((float)3));
//        Test_Y.append( ((float)0));
//        Test_X.append( ((float)3));
//        Test_Y.append( ((float)7));
//        Graph2->setData(Test_X,Test_Y);
    /***********************************/


    UIplotP->xAxis->setBasePen(QPen(Qt::white, 1));
    UIplotP->yAxis->setBasePen(QPen(Qt::white, 1));
    UIplotP->xAxis->setTickPen(QPen(Qt::white, 1));
    UIplotP->yAxis->setTickPen(QPen(Qt::white, 1));
    UIplotP->xAxis->setSubTickPen(QPen(Qt::white, 1));
    UIplotP->yAxis->setSubTickPen(QPen(Qt::white, 1));
    UIplotP->xAxis->setTickLabelColor(Qt::white);
    UIplotP->yAxis->setTickLabelColor(Qt::white);
    UIplotP->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    UIplotP->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    UIplotP->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    UIplotP->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    UIplotP->xAxis->grid()->setSubGridVisible(true);
    UIplotP->yAxis->grid()->setSubGridVisible(true);
    UIplotP->xAxis->grid()->setZeroLinePen(Qt::NoPen);
    UIplotP->yAxis->grid()->setZeroLinePen(Qt::NoPen);
    UIplotP->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
    UIplotP->yAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);



    QLinearGradient plotGradient;
    plotGradient.setStart(0, 0);
    plotGradient.setFinalStop(0, 350);
    plotGradient.setColorAt(0, QColor(5, 5, 5));
    plotGradient.setColorAt(1, QColor(5, 5, 5));
    UIplotP->setBackground(plotGradient);
    QLinearGradient axisRectGradient;
    axisRectGradient.setStart(0, 0);
    axisRectGradient.setFinalStop(0, 350);
    axisRectGradient.setColorAt(0, QColor(5, 5, 5));
    axisRectGradient.setColorAt(1, QColor(5, 5, 5));
    UIplotP->axisRect()->setBackground(axisRectGradient);


//    UIplotP->graph()->setName(QString("New graph %1").arg(UIplotP->graphCount()-1));

    UIplotP->replot();
    UIplotP->update();

}


GenericQCP::~GenericQCP()
{
//    qDebug() << "Plot X Deconstructed";

    if(Graph1){
            delete Graph1;      }

    if(Graph2){
              delete Graph2;  }
    if(Graph3){
              delete Graph2;  }
    if(Graph4){
              delete Graph2;  }
    if(Graph5){
              delete Graph2;  }
    if(Graph6){
              delete Graph2;  }
    if(SelectedPointMarkerGraphX){
              delete SelectedPointMarkerGraphX;  }
    if(SelectedPointMarkerGraphY){
              delete SelectedPointMarkerGraphY;  }
}



void GenericQCP::Graph_AppendData(float X_Pos1,float Y_Pos1,
                                        float X_Pos2,float Y_Pos2,
                                        float X_Pos3,float Y_Pos3,
                                        float X_Pos4,float Y_Pos4,
                                        float X_Pos5,float Y_Pos5,
                                        float X_Pos6,float Y_Pos6)
{
    DataVector_X1.append( ((float)X_Pos1));
    DataVector_Y1.append( ((float)Y_Pos1));
    DataVector_X2.append( ((float)X_Pos2));
    DataVector_Y2.append( ((float)Y_Pos2));
    DataVector_X3.append( ((float)X_Pos3));
    DataVector_Y3.append( ((float)Y_Pos3));
    DataVector_X4.append( ((float)X_Pos4));
    DataVector_Y4.append( ((float)Y_Pos4));
    DataVector_X5.append( ((float)X_Pos5));
    DataVector_Y5.append( ((float)Y_Pos5));
    DataVector_X6.append( ((float)X_Pos6));
    DataVector_Y6.append( ((float)Y_Pos6));
}

void GenericQCP::Graph_UpdateReplot(void)
{
    Graph1->setData(DataVector_X1,DataVector_Y1);
    Graph2->setData(DataVector_X2,DataVector_Y2);
    Graph3->setData(DataVector_X3,DataVector_Y3);
    Graph4->setData(DataVector_X4,DataVector_Y4);
    Graph5->setData(DataVector_X5,DataVector_Y5);
    Graph6->setData(DataVector_X6,DataVector_Y6);

    uint32_t VectorXSize = DataVector_X1.size();
    if(VectorXSize > 0)
    {
      if(VectorXSize > 300)
      {
          UIplotP->xAxis->setRange
              ( *std::min_element(DataVector_X1.end() - 300,DataVector_X1.end() ) -1,
               *std::max_element(DataVector_X1.end() - 300,DataVector_X1.end() ) +1);
      }
      else{
          UIplotP->xAxis->setRange
              ( *std::min_element(DataVector_X1.begin(),DataVector_X1.end() ) -1,
               *std::max_element(DataVector_X1.begin(),DataVector_X1.end() ) +1);
      }

      UIplotP->yAxis->setRange
          (  *std::min_element(DataVector_Y1.begin() ,DataVector_Y1.end() ) -1,
           *std::max_element(DataVector_Y1.begin() ,DataVector_Y1.end() ) +2 );

      UIplotP->replot();
      //    UIplotP->update();
    }
}

void GenericQCP::Graph_ClearData(void)
{
    DataVector_X1.clear();
    DataVector_Y1.clear();

    DataVector_X2.clear();
    DataVector_Y2.clear();

    DataVector_X3.clear();
    DataVector_Y3.clear();

    DataVector_X4.clear();
    DataVector_Y4.clear();

    DataVector_X5.clear();
    DataVector_Y5.clear();

    DataVector_X6.clear();
    DataVector_Y6.clear();

    DataVector_PointX_x.clear();
    DataVector_PointX_y.clear();

    DataVector_PointY_x.clear();
    DataVector_PointY_y.clear();

    Graph1->setData(DataVector_X1,DataVector_Y1);
    Graph2->setData(DataVector_X2,DataVector_Y2);
    Graph3->setData(DataVector_X3,DataVector_Y3);
    Graph4->setData(DataVector_X4,DataVector_Y4);
    Graph5->setData(DataVector_X5,DataVector_Y5);
    Graph6->setData(DataVector_X6,DataVector_Y6);

    SelectedPointMarkerGraphY->setData(DataVector_PointY_x,DataVector_PointY_y);
    SelectedPointMarkerGraphX->setData(DataVector_PointX_x,DataVector_PointX_y);

//    UIplotP->xAxis->setRange
//        ( *std::min_element(DataVector_X1.begin(),DataVector_X1.end() ) -1,
//         *std::max_element(DataVector_X1.begin(),DataVector_X1.end() ) +1);

//    UIplotP->yAxis->setRange
//        (  *std::min_element(DataVector_Y1.begin() ,DataVector_Y1.end() ) -1,
//         *std::max_element(DataVector_Y1.begin() ,DataVector_Y1.end() ) +2 );

    UIplotP->replot();
    UIplotP->update();
}


void GenericQCP::Graph_axisLabelDoubleClick(QCPAxis *axis, QCPAxis::SelectablePart part)
{
    // Set an axis label by double clicking on it
    if (part == QCPAxis::spAxisLabel) // only react when the actual axis label is clicked, not tick label or axis backbone
    {
        bool ok;
        QString newLabel = QInputDialog::getText(this, "QCustomPlot example", "New axis label:", QLineEdit::Normal, axis->label(), &ok);
        if (ok)
        {
            axis->setLabel(newLabel);
            UIplotP->replot();
        }
    }
}

void GenericQCP::Graph_legendDoubleClick(QCPLegend *legend, QCPAbstractLegendItem *item)
{
    // Rename a graph by double clicking on its legend item
    Q_UNUSED(legend)
    if (item) // only react if item was clicked (user could have clicked on border padding of legend where there is no item, then item is 0)
    {
        QCPPlottableLegendItem *plItem = qobject_cast<QCPPlottableLegendItem*>(item);
        bool ok;
        QString newName = QInputDialog::getText(this, "QCustomPlot example", "New graph name:", QLineEdit::Normal, plItem->plottable()->name(), &ok);
        if (ok)
        {
            plItem->plottable()->setName(newName);
            UIplotP->replot();
        }
    }
}

void GenericQCP::Graph_selectionChanged()
{
    /*
   normally, axis base line, axis tick labels and axis labels are selectable separately, but we want
   the user only to be able to select the axis as a whole, so we tie the selected states of the tick labels
   and the axis base line together. However, the axis label shall be selectable individually.

   The selection state of the left and right axes shall be synchronized as well as the state of the
   bottom and top axes.

   Further, we want to synchronize the selection of the graphs with the selection state of the respective
   legend item belonging to that graph. So the user can select a graph by either clicking on the graph itself
   or on its legend item.
  */

    // make top and bottom axes be selected synchronously, and handle axis and tick labels as one selectable object:
    if (UIplotP->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || UIplotP->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
        UIplotP->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || UIplotP->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
    {
        UIplotP->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
        UIplotP->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    }
    // make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
    if (UIplotP->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || UIplotP->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
        UIplotP->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || UIplotP->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
    {
        UIplotP->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
        UIplotP->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    }

    // synchronize selection of graphs with selection of corresponding legend items:
    for (int i=0; i<UIplotP->graphCount(); ++i)
    {
        QCPGraph *graph = UIplotP->graph(i);
        QCPPlottableLegendItem *item = UIplotP->legend->itemWithPlottable(graph);
        if (item->selected() || graph->selected())
        {
            item->setSelected(true);
            graph->setSelection(QCPDataSelection(graph->data()->dataRange()));
        }
    }
}

void GenericQCP::Graph_mousePress()
{
    // if an axis is selected, only allow the direction of that axis to be dragged
    // if no axis is selected, both directions may be dragged

    if (UIplotP->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
        UIplotP->axisRect()->setRangeDrag(UIplotP->xAxis->orientation());
    else if (UIplotP->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
        UIplotP->axisRect()->setRangeDrag(UIplotP->yAxis->orientation());
    else
        UIplotP->axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);
}

void GenericQCP::Graph_mouseWheel()
{
    // if an axis is selected, only allow the direction of that axis to be zoomed
    // if no axis is selected, both directions may be zoomed

    if (UIplotP->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
        UIplotP->axisRect()->setRangeZoom(UIplotP->xAxis->orientation());
    else if (UIplotP->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
        UIplotP->axisRect()->setRangeZoom(UIplotP->yAxis->orientation());
    else
        UIplotP->axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);
}

void GenericQCP::Graph_addRandomGraph()
{
    int n = 50; // number of points in graph
    double xScale = (std::rand()/(double)RAND_MAX + 0.5)*2;
    double yScale = (std::rand()/(double)RAND_MAX + 0.5)*2;
    double xOffset = (std::rand()/(double)RAND_MAX - 0.5)*4;
    double yOffset = (std::rand()/(double)RAND_MAX - 0.5)*10;
    double r1 = (std::rand()/(double)RAND_MAX - 0.5)*2;
    double r2 = (std::rand()/(double)RAND_MAX - 0.5)*2;
    double r3 = (std::rand()/(double)RAND_MAX - 0.5)*2;
    double r4 = (std::rand()/(double)RAND_MAX - 0.5)*2;    QVector<double> x(n), y(n);
    for (int i=0; i<n; i++)
    {
        x[i] = (i/(double)n-0.5)*10.0*xScale + xOffset;
        y[i] = (qSin(x[i]*r1*5)*qSin(qCos(x[i]*r2)*r4*3)+r3*qCos(qSin(x[i])*r4*2))*yScale + yOffset;
    }

    UIplotP->addGraph();
    UIplotP->graph()->setName(QString("New graph %1").arg(UIplotP->graphCount()-1));
    UIplotP->graph()->setData(x, y);
    UIplotP->graph()->setLineStyle((QCPGraph::LineStyle)(std::rand()%5+1));
    if (std::rand()%100 > 50)
        UIplotP->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(std::rand()%14+1)));
    QPen graphPen;
    graphPen.setColor(QColor(std::rand()%245+10, std::rand()%245+10, std::rand()%245+10));
    graphPen.setWidthF(std::rand()/(double)RAND_MAX*2+1);
    UIplotP->graph()->setPen(graphPen);
    UIplotP->replot();
}

void GenericQCP::Graph_removeSelectedGraph()
{
    if (UIplotP->selectedGraphs().size() > 0)
    {
        UIplotP->removeGraph(UIplotP->selectedGraphs().first());
        UIplotP->replot();
    }
}

void GenericQCP::Graph_removeAllGraphs()
{
    UIplotP->clearGraphs();
    UIplotP->replot();
}

void GenericQCP::Graph_contextMenuRequest(QPoint pos)
{
    QMenu *menu = new QMenu(this);
    menu->setAttribute(Qt::WA_DeleteOnClose);

    if (UIplotP->legend->selectTest(pos, false) >= 0) // context menu on legend requested
    {
        menu->addAction("Move to top left", this, SLOT(Graph_moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignLeft));
        menu->addAction("Move to top center", this, SLOT(Graph_moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignHCenter));
        menu->addAction("Move to top right", this, SLOT(Graph_moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignRight));
        menu->addAction("Move to bottom right", this, SLOT(Graph_moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignRight));
        menu->addAction("Move to bottom left", this, SLOT(Graph_moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignLeft));
    } else  // general context menu on graphs requested
    {
//        menu->addAction("Add random graph", this, SLOT(Graph_addRandomGraph()));
//        if (UIplotP->selectedGraphs().size() > 0)
//            menu->addAction("Remove selected graph", this, SLOT(Graph_removeSelectedGraph()));
//        if (UIplotP->graphCount() > 0)
//            menu->addAction("Remove all graphs", this, SLOT(Graph_removeAllGraphs()));
    }

    menu->popup(UIplotP->mapToGlobal(pos));
}

void GenericQCP::Graph_moveLegend()
{
    if (QAction* contextAction = qobject_cast<QAction*>(sender())) // make sure this slot is really called by a context menu action, so it carries the data we need
    {
        bool ok;
        int dataInt = contextAction->data().toInt(&ok);
        if (ok)
        {
            UIplotP->axisRect()->insetLayout()->setInsetAlignment(0, (Qt::Alignment)dataInt);
            UIplotP->replot();
        }
    }
}

void GenericQCP::Graph_graphClicked(QCPAbstractPlottable *plottable, int dataIndex)
{
    double ValY = plottable->interface1D()->dataMainValue(dataIndex);
    double ValX = plottable->interface1D()->dataMainKey(dataIndex);

    uint32_t resVectorDataIndex = 0;
    uint32_t DataSizeGr1 = Graph1->data()->size();

    if(DataSizeGr1 > 0)
    {
        for(int i=0; i<DataSizeGr1; i++)
        {
            if(DataVector_X1.at(i) == ValX && DataVector_Y1.at(i) == ValY)
            {
                resVectorDataIndex = i;
                break;
            }
            if(DataVector_X2.at(i) == ValX && DataVector_Y2.at(i) == ValY)
            {
                resVectorDataIndex = i;
                break;
            }
            if(DataVector_X3.at(i) == ValX && DataVector_Y3.at(i) == ValY)
            {
                resVectorDataIndex = i;
                break;
            }
        }
    }

    QString message = QString("Clicked on graph '%1' at data point #%2 with value ValX %3: ValY %4: VecDataIndex %5")
                          .arg(plottable->name()).arg(dataIndex).arg(ValX).arg(ValY).arg(resVectorDataIndex);
    qDebug() << message;

    if(resVectorDataIndex > 1)
    {
        emit LfGraphSignal_graphClicked(resVectorDataIndex-1);
    }
}


void GenericQCP::Graph_DrawMarkersAtDataIndex(int DataIndex)
{
    /*Align to Graph1 it\s best idea what I figured out*/
//    double dataValueVer =    Graph1->dataMainKey(DataIndex);
        double dataValueVer =    DataVector_X1.at(DataIndex);
    double dataValueHor =    DataVector_Y1.at(DataIndex);

//        DataVector_X1, DataVector_Y1;

    if(DataIndex != 0)
    {
        {
            QVector<double>  Test_X;
            QVector<double>  Test_Y;
            Test_X.append( ((float)dataValueVer));
            Test_Y.append( ((float)-5000.0F));
            Test_X.append( ((float)dataValueVer));
            Test_Y.append( ((float)5000.0F ));
            SelectedPointMarkerGraphX->setData(Test_X,Test_Y);
        }

        {
            /*Draw horizontal line*/
            //Graph2->set
            QVector<double>  Test_X;
            QVector<double>  Test_Y;
            Test_X.append( ((float) -5000.0F));
            Test_Y.append( ((float)dataValueHor ));
            Test_X.append( ((float) 5000.0F));
            Test_Y.append( ((float)dataValueHor ));
            SelectedPointMarkerGraphY->setData(Test_X,Test_Y);
        }
    }
    else{
        QVector<double>  Test_X;
        QVector<double>  Test_Y;
        SelectedPointMarkerGraphX->setData(Test_X,Test_Y);
        SelectedPointMarkerGraphY->setData(Test_X,Test_Y);
    }


    Graph_UpdateReplot(); //?? is need?
}
