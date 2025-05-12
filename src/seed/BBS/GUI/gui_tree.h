/* 
 * File:   gui.h
 * Author: hargalaten
 *
 * Created on 23 luglio 2015, 12.58
 */

#ifndef GUI_TREE_H
#define	GUI_TREE_H

#include "seed.h"

#include <qt5/QtWidgets/QApplication>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QProgressBar>
#include <qt5/QtWidgets/QSlider>

//examples from http://zetcode.com/gui/qt5/eventsandsignals/

//quit on click

#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtGui/QKeyEvent>
#include <qt5/QtWidgets/QMainWindow>
#include <qt5/QtWidgets/QStatusBar>
#include <qt5/QtWidgets/QFrame>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QLineEdit>
#include <qt5/QtWidgets/QTreeWidget>
#include <qt5/QtWidgets/QGroupBox>
#include <qt5/QtWidgets/QMenu>
#include <qt5/QtWidgets/QTextEdit>
#include <qt5/QtCore/QTimer>
#include <qt5/QtWidgets/QListWidget>
#include <qt5/QtWidgets/QComboBox>

#include <qt5/QtWidgets/QTableWidget>
#include <qt5/QtWidgets/QHeaderView>
#include <qt5/QtWidgets/QMessageBox>

#include <qt5/QtCore/QProcess>

#include <qt5/QtCore/QXmlStreamReader>

//#include <qt5/QtWidgets/QGraphicsWidget>
#include <QtCharts/QChart> //needs libqt5charts5-dev
#include <QtCharts/QLineSeries>
#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>


//this was created starting from:
//  https://doc.qt.io/qt-5/qtwidgets-graphicsview-elasticnodes-example.html

//distances between nodes
#define GUI_TREE_X_STEP 350 //300
#define GUI_TREE_Y_STEP 110 //100

//size of the single node
#define GUI_TREE_X_NODE 200
#define GUI_TREE_Y_NODE 70

//pixels to plot a character
#define GUI_TREE_X_CHAR2PX 10


using namespace QtCharts;

using namespace seed;

class Edge;

class GraphWidget;

inline void drawContents(QPainter * p, QTextDocument & doc, int off_x, int off_y) {
    p->save();
    p->translate(off_x,off_y);
    //doc.setTextWidth(rect.width());
    doc.drawContents(p); //doc.drawContents(p, rect);
    p->restore();
}

enum GUINodeType { GUI_NODE_WMN, GUI_NODE_CON, GUI_NODE_BEH, GUI_NODE_VAR };

class Node : public QGraphicsItem
{
public:
    Node(GraphWidget *graphWidget, GUINodeType n_type);

    void addEdge(Edge *edge);
    QVector<Edge *> edges() const;

    enum { Type = UserType + 1 };
    int type() const override { return Type; }
    
    void calculateForces();
    bool advancePosition(); 

    QRectF boundingRect() const override;
    //QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    
    inline QPointF getPos(){ return newPos; };
    
    void setGraphicalDimensions(QPainter *painter); //set px_* dimensions
    
    void addSon(QGraphicsScene *scene, Node *new_son);
    void removeSon(QGraphicsScene *scene, Node *old_son);

    GUINodeType node_type; //wm_node, contribution, behavior, variable, etc.
    
    
    //node_type: GUI_NODE_WMN
    
    void loadWMN(WM_node *node);
    
    void paint_WMN(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    
    QString search_node(QString toFind);
    
    std::string wm_instance;
    QString instance;
    //std::vector<QString> releaser;
    QTextDocument releaser,goal; //qdoc has HTML tags! //QString releaser,goal;
    
    double emp;
    double rtm;
    bool abstract;
    
    double w_father;
    
    bool releaserStatus, goalStatus;
    bool brenchReleaserStatus;
    
    
    //node_type: GUI_NODE_CON
    
    void loadCON(std::string, Node *);
    
    void paint_CON(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    
    Node * target_node;
    
    
    //node_type: GUI_NODE_VAR
    
    QString var_value;
    
    QString winning_node_instance;
    
    std::vector<Node *> contributing_nodes;
    
    
    
    QString max_node_string;
    
    //int level; //level of depth in the hierarchy
    int px_segment; //segment of space needed by this node to be visualized
    
    int depth; //len of the subtree
    
    int px_x_node; //dimensions of the ellipse 
    int px_y_node;
    
    int px_x_bb; //space needed by the node to be visualized 
    int px_y_bb;
    
    int px_tree_x; //space needed by the subtree (rooted in this node) to be visualized
    int px_tree_y;
    
    int px_target_pos_x; //target position of the node in the scene
    int px_target_pos_y;
    
    std::vector<Node *> son;
    
protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    
    bool releaser2qstring(WM_node *node, QString *qs);
    bool goal2qstring(WM_node *node, QString *qs);
    

private:
    QVector<Edge *> edgeList;
    QPointF newPos;
    GraphWidget *graph;
};



class Edge : public QGraphicsItem
{
public:
    Edge(Node *sourceNode, Node *destNode);

    Node *sourceNode() const;
    Node *destNode() const;

    void adjust();

    enum { Type = UserType + 2 };
    int type() const override { return Type; }

protected:
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    
    void paint_WMN(QPainter *painter);
    
    void paint_CON(QPainter *painter);

private:
    Node *source, *dest;

    QPointF sourcePoint;
    QPointF destPoint;
    qreal arrowSize = 10;
};



class GraphWidget : public QGraphicsView
{
    //Q_OBJECT

public:
    GraphWidget(QWidget *parent = nullptr);

    void itemMoved();
    
    void loadFromWM(QGraphicsScene *scene, WM_node *node, Node *father_qt = NULL);
    
    void showContributions(Node *n, WM_node *node);
    
    void setPositionWMN(Node *root_qt, int x, int y);
    
    void setPositionCON();
    
    void updateFromWM(QGraphicsScene *scene, WM_node *wm_node, Node *qt_node);
    
    void removeContributions(QGraphicsScene *scene, Node *node_qt);
    
    void removeSubtree(QGraphicsScene *scene, Node *root_qt);
    
    QRectF subtree_boundingRect(Node *node_qt);

//public slots:
//    void shuffle();
//    void zoomIn();
//    void zoomOut();

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void timerEvent(QTimerEvent *event) override;
#if QT_CONFIG(wheelevent)
    void wheelEvent(QWheelEvent *event) override;
#endif
    
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    
    //void drawBackground(QPainter *painter, const QRectF &rect) override;

    void scaleView(qreal scaleFactor);

private:
    int timerId = 0;
    
    Node *centerNode; //root of the WMN tree
    
    std::vector<Node *> contNodes; //list of contribution-nodes
    
    std::vector<Node *> varNodes; //list of variable-nodes (contended)
    
    int px_width;
    int px_height;
    
    int lev_step;
    
    QPointF m_ori_pos;
    QPointF m_new_pos;
};

#endif	/* GUI_TREE_H */

