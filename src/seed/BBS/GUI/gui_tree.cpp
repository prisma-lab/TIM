/* 
 * File:   gui.h
 * Author: hargalaten
 *
 * Created on 23 luglio 2015, 12.58
 */


//#include "../../seed.h"

#include "GUI/gui_tree.h"

Node::Node(GraphWidget *graphWidget, GUINodeType n_type)
    : graph(graphWidget)
{
    setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setCacheMode(DeviceCoordinateCache);
    setZValue(-1);
    
    node_type = n_type;
}

//unused
//QString Node::search_node(QString toFind){
//    return QString();
//}

bool Node::releaser2qstring(WM_node *node, QString *qs){
    bool nodeReleased = true;
    qs->clear();
    //for each element of the releaser
    for(size_t i=0; i<node->releaser.size(); i++){
        
        QString qstr = QString::fromStdString( node->releaser[i] );
        if(qstr.at(0) == '-')
            qstr.replace(0,1,QChar(0x00AC));
        
        //if the element is false plot it as red
        //if( (node->releaser[i].at(0) != '-' && !WMV.get<bool>(node->releaser[i])) ||
        //    (node->releaser[i].at(0) == '-' && WMV.get<bool>(node->releaser[i])) ){
        if(!node->eval(node->releaser[i])){
            nodeReleased = false;

            //ss<<"<font color=\"red\">"<<node->releaser[i]<<"</font>"; //html not enabled in qtree
            //ss<<"<u>"<<node->releaser[i]<<"</u>"; //html not enabled in qtree
            //ss<<"~"<<node->releaser[i];
            
            //qs->append("<i>");
            qs->append("<font color=\"Gray\">");
            qs->append(qstr);
            //qs->append("</i>");
            qs->append("</font>");
        }
        //otherwise (is true) plot it as green
        else{
            //ss<<"<font color=\"green\">"<<node->releaser[i]<<"</font>";
            qs->append("<font color=\"Black\">");
            qs->append(qstr);
            qs->append("</font>");
        }

        //if there are other elements left, add an end line
        if(i != node->releaser.size()-1){
            qs->append("<font color=\"Black\"> ");
            qs->append( QChar(0x0245) );
            qs->append(" </font>");
        }
    }

    if(node->releaser.size() == 0)
        *qs = "TRUE";

    return nodeReleased;
}
    
bool Node::goal2qstring(WM_node *node, QString *qs){

    bool nodeGoal = true;
    // added in SEED 6.0
    if(node->goal.size() == 0)
        // set goal to false if the node has no goal
        nodeGoal = false;

    qs->clear();
    //for each element of the goal
    for(size_t i=0; i<node->goal.size(); i++){
        
        QString qstr = QString::fromStdString( node->goal[i] );
        if(qstr.at(0) == '-')
            qstr.replace(0,1,QChar(0x00AC));
        
        // added in SEED 6.0
        std::string goal_str = node->goal[i];
        char c;
        if(node->goal[i].at(0) == '-'){
            std::stringstream ss(goal_str);
            //discard the negation
            ss>>c>>goal_str;
        }


        //if the element is false plot it as black
        //if( (node->goal[i].at(0) != '-' && !WMV.get<bool>(goal_str)) ||
        //    (node->goal[i].at(0) == '-' && WMV.get<bool>(goal_str)) ){
        if(!node->eval(node->goal[i])){
            nodeGoal = false;

            //ss<<"<font color=\"black\">"<<node->releaser[i]<<"</font>"; //html not enabled in qtree
            //ss<<"~"<<node->goal[i];
            
            //qs->append("<i>");
            qs->append("<font color=\"Gray\">"); //qs->append("<font color=\"DarkGray\">");
            qs->append(qstr);
            //qs->append("</i>");
            qs->append("</font>");
        }
        //otherwise (is true) plot it as blue
        else{
            //ss<<"<font color=\"blue\">"<<node->releaser[i]<<"</font>";
            qs->append("<font color=\"Black\">");
            qs->append(qstr);
            qs->append("</font>");
        }

        //if there are other elements left, add an end line
        if(i != node->goal.size()-1){
            // modified in SEED 6.0
            //  NOTE: the goal's elements are in AND as releaser's ones
            //qs->append(" V ");
            qs->append("<font color=\"Black\"> ");
            qs->append( QChar(0x0245) );
            qs->append(" </font>");
        }
    }


    if(node->goal.size() == 0)
        *qs = "";

    return nodeGoal;
}


void Node::loadWMN(WM_node *node){

    wm_instance = node->instance;

    //adjust the instance of some nodes to be better visualized
    //instance = QString::fromStdString( node->instance );
    std::vector<std::string> iv = instance2vector(node->instance);
    if(iv[0] == "compete"){
        instance = QString::fromStdString( "EXE(" + iv[3] + ")" );
    } 
    else if(iv[0] == "state"){
        instance = QString::fromStdString( "COM(" + iv[1] + ")" );
    }
    else if(iv[0] == "remember" && instance2vector(iv[1])[0] == "state"){
        instance = QString::fromStdString( "MEM(" + instance2vector(iv[1])[1] + ")" );
    }
    else
        instance = QString::fromStdString( node->instance );

    
    brenchReleaserStatus = node->isBranchReleased();
    
    QString html_releaer, html_goal;
    releaserStatus = releaser2qstring(node,&html_releaer);
    goalStatus = goal2qstring(node,&html_goal);
    
    releaser.setHtml(html_releaer);
    goal.setHtml(html_goal);
    
    abstract = node->abstract;
    
    emp = node->emphasis();

    rtm = node->rtm;
    
    px_x_node = GUI_TREE_X_NODE >= instance.length() * GUI_TREE_X_CHAR2PX ? GUI_TREE_X_NODE : instance.length() * GUI_TREE_X_CHAR2PX;
    px_y_node = GUI_TREE_Y_NODE;
    
    max_node_string = releaser.toPlainText().length() >= instance.length() ? releaser.toPlainText() : instance;
    max_node_string = max_node_string.length() >= goal.toPlainText().length() ? max_node_string : goal.toPlainText();
    
    px_x_bb = max_node_string.length() * GUI_TREE_X_CHAR2PX >= px_x_node ? max_node_string.length() * GUI_TREE_X_CHAR2PX : px_x_node;
    px_y_bb = GUI_TREE_Y_NODE;
    
    w_father = nanf("");
    
    if(node->father != NULL){
        double *w = WMV.get< std::unordered_map<std::string, double*> >(node->name + ".weights")[node->father->name];
        if(w != NULL)
            w_father = *w;
    }
    
}

void Node::loadCON(std::string n_instance, Node *t_node){
    
    instance = QString::fromStdString( n_instance );
    
    target_node = t_node;
    
//    px_x_node = GUI_TREE_X_NODE >= instance.length() * GUI_TREE_X_CHAR2PX ? GUI_TREE_X_NODE : instance.length() * GUI_TREE_X_CHAR2PX;
//    px_y_node = GUI_TREE_Y_NODE/2;
    
    px_x_node = 10; //GUI_TREE_X_NODE >= instance.length() * GUI_TREE_X_CHAR2PX ? GUI_TREE_X_NODE : instance.length() * GUI_TREE_X_CHAR2PX;
    px_y_node = 10; //GUI_TREE_Y_NODE/2;
    
    max_node_string = instance;
    
    px_x_bb = GUI_TREE_X_NODE >= instance.length() * GUI_TREE_X_CHAR2PX ? GUI_TREE_X_NODE : instance.length() * GUI_TREE_X_CHAR2PX; //px_x_node;
    px_y_bb = GUI_TREE_Y_NODE/2;
    
}

void Node::addSon(QGraphicsScene *scene, Node *new_son){
    son.push_back(new_son);
    
    scene->addItem(new_son);
            
    Edge *e = new Edge(this, new_son);
    scene->addItem(e);
}

//FIXED on 14/06/2023 to solve crashes after node removal
void Node::removeSon(QGraphicsScene *scene, Node *old_son){
    int idx = -1;
    for(size_t i=0; i<son.size(); i++){
        //std::cout<<"\tchecking son "<<son[i]->instance.toStdString()<<std::endl;
        if(son[i] == old_son){
            //std::cout<<"\t\tthis son has to be removed!"<<std::endl;
            idx = (int) i;
        }
    }
    
    if(idx >= 0){
        
        Edge *old_edge;
        
        //for(auto i=0; i<edges().size(); i++){
        int i = 0;
        while(i<edgeList.size()){
            if(edgeList[i]->destNode() == old_son){
                old_edge = edgeList[i];
                edgeList.remove(i);
                //std::cout<<"\t\t\tremoving edge "<<old_edge<<std::endl;
                scene->removeItem(old_edge);
                delete old_edge;
                //break;
                //std::cout<<"\t\t\tdone "<<std::endl;
                continue;
            }
            i++;
        }
        
        scene->removeItem(old_son);
        delete old_son;

        son.erase(son.begin()+idx);
    }
}


void Node::addEdge(Edge *edge)
{
    edgeList << edge;
    edge->adjust();
}

QVector<Edge *> Node::edges() const
{
    return edgeList;
}



void Node::calculateForces()
{
    if (!scene() || scene()->mouseGrabberItem() == this) {
        newPos = pos();
        return;
    }
    
    // Sum up all forces pushing this item away
    qreal xvel = 0;
    qreal yvel = 0;
//    const QList<QGraphicsItem *> items = scene()->items();
//    for (QGraphicsItem *item : items) {
//        Node *node = qgraphicsitem_cast<Node *>(item);
//        if (!node)
//            continue;
//
//        QPointF vec = mapToItem(node, 0, 0);
//        qreal dx = vec.x();
//        qreal dy = vec.y();
//        double l = 2.0 * (dx * dx + dy * dy);
//        if (l > 0) {
//            xvel += (dx * 150.0) / l;
//            yvel += (dy * 150.0) / l;
//        }
//    }
//    
//    // Now subtract all forces pulling items together
//    double weight = (edgeList.size() + 1) * 10;
//    for (const Edge *edge : qAsConst(edgeList)) {
//        QPointF vec;
//        if (edge->sourceNode() == this)
//            vec = mapToItem(edge->destNode(), 0, 0);
//        else
//            vec = mapToItem(edge->sourceNode(), 0, 0);
//        xvel -= vec.x() / weight;
//        yvel -= vec.y() / weight;
//    }
//    
//    if (qAbs(xvel) < 0.1 && qAbs(yvel) < 0.1)
//        xvel = yvel = 0;
    
    QRectF sceneRect = scene()->sceneRect();
    newPos = pos() + QPointF(xvel, yvel);
    newPos.setX(qMin(qMax(newPos.x(), sceneRect.left() + px_x_node/2), sceneRect.right() - px_x_node/2));
    newPos.setY(qMin(qMax(newPos.y(), sceneRect.top() + px_y_node/2), sceneRect.bottom() - px_y_node/2));
}


bool Node::advancePosition()
{
    if (newPos == pos())
        return false;

    setPos(newPos);
    return true;
}

QRectF Node::boundingRect() const
{
    qreal adjust = 10;//2;
    //initial point (left-down) and lengths
    return QRectF( -px_x_bb/2 - adjust, -px_y_bb/2 - adjust, px_x_bb + 5 + adjust, px_y_bb + 10 + adjust);
}


//paint selector, based on type
void Node::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *qw){
    switch(node_type)
    {
        case GUINodeType::GUI_NODE_WMN : paint_WMN(painter, option, qw); break;
        case GUINodeType::GUI_NODE_CON : paint_CON(painter, option, qw); break;
        default: break;
        //case blue : std::cout << "blue\n";  break;
    }
}


void Node::paint_WMN(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    (void) option; //unused
    QFontMetrics fm = painter->fontMetrics(); //get dimensions of charts with the current font
    int txt_y_px = fm.height() + 2;
    
    
    //std::cout<<"max string: "<<max_node_string.toStdString()<<std::endl;
    
    //std::cout<<instance.toStdString()<<" bb len: "<<px_x_bb<<std::endl;
    
    //painter->setPen(Qt::NoPen);
    
    if(goalStatus)
        painter->setPen(QPen(Qt::blue, 5));
    else if(brenchReleaserStatus)
        painter->setPen(QPen(Qt::green, 5));
    else
        painter->setPen(QPen(Qt::red, 5));
    //painter->setPen(QPen(Qt::black, 0));
    
    if(abstract)
        painter->setBrush(Qt::lightGray);
    else
        painter->setBrush(Qt::darkGray);
    
    //initial point (left-down) and lengths
    painter->drawEllipse(-px_x_node/2, -px_y_node/2, px_x_node, px_y_node);

//    QRadialGradient gradient(-3, -3, GUI_TREE_X_NODE/2);
//    if (option->state & QStyle::State_Sunken) {
//        gradient.setCenter(3, 3);
//        gradient.setFocalPoint(3, 3);
//        gradient.setColorAt(1, QColor(Qt::gray).lighter(120));
//        gradient.setColorAt(0, QColor(Qt::darkGray).lighter(120));
//    } else {
//        gradient.setColorAt(0, Qt::gray);
//        gradient.setColorAt(1, Qt::darkGray);
//    }
//    painter->setBrush(gradient);
//
//    painter->setPen(QPen(Qt::black, 0));
//    painter->drawEllipse(-GUI_TREE_X_NODE/2, -GUI_TREE_Y_NODE/2, GUI_TREE_X_NODE, GUI_TREE_Y_NODE);
    
    //text
    painter->setPen(QPen(Qt::black, 0));
    //painter->drawText(boundingRect(), Qt::AlignCenter, instance);
    
    std::stringstream ss;
    ss<<instance.toStdString() << "\n(" << roundec(emp,3) << ")";
    painter->drawText(boundingRect(), Qt::AlignCenter, QString::fromStdString( ss.str() ));
    
    
    //draw releaser
    int rel_x_px = fm.width(releaser.toPlainText()) + 4;
    
    if(releaserStatus)
        painter->setBrush(Qt::green);
    else
        painter->setBrush(Qt::red);
    
    QRectF rel_rect(-rel_x_px/2, -txt_y_px -txt_y_px -5 , rel_x_px, txt_y_px);
    painter->drawRoundRect(rel_rect, 4,4);
    
    //painter->drawText(rel_rect, Qt::AlignCenter, releaser);
    drawContents(painter, releaser, rel_rect.left()-1, rel_rect.top()-3 );
    
    //draw goal
    if(goal.toPlainText() != ""){
        int goal_x_px = fm.width(goal.toPlainText()) + 4;

        painter->setBrush(Qt::cyan);

        QRectF goal_rect(-goal_x_px/2, +txt_y_px +txt_y_px +5 , goal_x_px, -txt_y_px);
        painter->drawRoundRect(goal_rect, 4,4);
        
        //painter->drawText(goal_rect, Qt::AlignCenter, goal);
        drawContents(painter, goal, -goal_x_px/2, +txt_y_px+1 ); //dont know why these measures!
    }
}


void Node::paint_CON(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    (void) option; //unused
    QFontMetrics fm = painter->fontMetrics(); //get dimensions of charts with the current font
    //int txt_y_px = fm.height() + 2;
    
    //std::cout<<"max string: "<<max_node_string.toStdString()<<std::endl;
    
    //std::cout<<instance.toStdString()<<" bb len: "<<px_x_bb<<std::endl;
    
    //painter->setPen(Qt::NoPen);
    
    painter->setPen(QPen(Qt::black, 1));
    
    painter->setBrush(Qt::black);
    
    //initial point (left-down) and lengths
    //painter->drawRect(-px_x_node/2, -px_y_node/2, px_x_bb, px_y_bb);
    painter->drawEllipse(-px_x_node/2, -px_y_node/2 -px_y_bb/2, px_x_node, px_y_node);
    
    //text
    painter->setPen(QPen(Qt::black, 0));
    //painter->drawText(boundingRect(), Qt::AlignCenter, instance);
    
    painter->drawText(boundingRect(), Qt::AlignCenter, instance );
    
}


QVariant Node::itemChange(GraphicsItemChange change, const QVariant &value)
{
    switch (change) {
    case ItemPositionHasChanged:
        for (Edge *edge : qAsConst(edgeList))
            edge->adjust();
        graph->itemMoved();
        break;
    default:
        break;
    };

    return QGraphicsItem::itemChange(change, value);
}

void Node::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    update();
    QGraphicsItem::mousePressEvent(event);
}

void Node::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}





//EDGES:



Edge::Edge(Node *sourceNode, Node *destNode)
    : source(sourceNode), dest(destNode)
{
    setAcceptedMouseButtons(Qt::NoButton);
    source->addEdge(this);
    dest->addEdge(this);
    adjust();
}

Node *Edge::sourceNode() const
{
    return source;
}

Node *Edge::destNode() const
{
    return dest;
}


void Edge::adjust()
{

    //std::cout<<source<<"->"<<dest<<std::endl;

    if (!source || !dest){
        //std::cout<<"return"<<std::endl;
        return;
    }
    
    //std::cout<<"  "<<source->instance.toStdString()<<"->"<<dest->instance.toStdString()<<std::endl;

    QLineF line(mapFromItem(source, 0, 0), mapFromItem(dest, 0, 0)); //this goes in SEGFAULT after node deallocation
    qreal length = line.length();

    //std::cout<<"done"<<std::endl;

    prepareGeometryChange();

    if (length > qreal(20.)) {
        QPointF src_edgeOffset((line.dx() * source->px_x_node/2) / length, (line.dy() * source->px_y_node/2) / length);
        QPointF dst_edgeOffset((line.dx() * dest->px_x_node/2) / length, (line.dy() * dest->px_y_node/2) / length);
        
        sourcePoint = line.p1() + src_edgeOffset;
        destPoint = line.p2() - dst_edgeOffset;
    } else {
        sourcePoint = destPoint = line.p1();
    }
}

QRectF Edge::boundingRect() const
{
    if (!source || !dest)
        return QRectF();

    qreal penWidth = 1;
    qreal extra = (penWidth + arrowSize) / 2.0;

    return QRectF(sourcePoint, QSizeF(destPoint.x() - sourcePoint.x(),
                                      destPoint.y() - sourcePoint.y()))
        .normalized()
        .adjusted(-extra, -extra, extra, extra);
}

//paint selector, based on source's type
void Edge::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{   
    switch(source->node_type)
    {
        case GUINodeType::GUI_NODE_WMN : paint_WMN(painter); break;
        case GUINodeType::GUI_NODE_CON : paint_CON(painter); break;
        default: break;
    }
}


void Edge::paint_WMN(QPainter *painter)
{
    
    if (!source || !dest)
        return;

    QLineF line(sourcePoint, destPoint);
    if (qFuzzyCompare(line.length(), qreal(0.)))
        return;
    
    // Draw the arrows
    double angle = std::atan2(-line.dy(), line.dx());
    
    QPointF destArrowP1 = destPoint + QPointF(sin(angle - M_PI / 3) * arrowSize,
                                              cos(angle - M_PI / 3) * arrowSize);
    QPointF destArrowP2 = destPoint + QPointF(sin(angle - M_PI + M_PI / 3) * arrowSize,
                                              cos(angle - M_PI + M_PI / 3) * arrowSize);
    
    // Draw the line itself
    if(source->goalStatus)
        painter->setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    else if(source->brenchReleaserStatus)
        painter->setPen(QPen(Qt::green, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    else
        painter->setPen(QPen(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    //painter->setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter->drawLine(line);
    

    if(source->goalStatus)
        painter->setBrush(Qt::blue);
    else if(source->brenchReleaserStatus)
        painter->setBrush(Qt::green);
    else
        painter->setBrush(Qt::red);
    //painter->setBrush(Qt::black);
    
    painter->drawPolygon(QPolygonF() << line.p2() << destArrowP1 << destArrowP2);
    //text
    painter->setPen(QPen(Qt::black, 0));
    //painter->drawText(boundingRect(), Qt::AlignCenter, instance);
    
    std::stringstream ss;
    
    //show also the weight as black number between brackets
    
    if(!isnan(dest->w_father)){
        ss<<"("<<roundec(dest->w_father,3)<<")";
        painter->drawText(boundingRect(), Qt::AlignCenter, QString::fromStdString( ss.str() ));
    }
}

void Edge::paint_CON(QPainter *painter)
{
    
    if (!source || !dest)
        return;

    QPointF new_sourcePoint = sourcePoint;//(source->px_target_pos_x, source->px_target_pos_y - (source->px_y_bb/2) );
    new_sourcePoint.setY( sourcePoint.y() - (source->px_y_bb/2));
    
    QLineF line(new_sourcePoint, destPoint);
    if (qFuzzyCompare(line.length(), qreal(0.)))
        return;
    
    // Draw the arrows
    double angle = std::atan2(-line.dy(), line.dx());
    
    QPointF destArrowP1 = destPoint + QPointF(sin(angle - M_PI / 3) * arrowSize,
                                              cos(angle - M_PI / 3) * arrowSize);
    QPointF destArrowP2 = destPoint + QPointF(sin(angle - M_PI + M_PI / 3) * arrowSize,
                                              cos(angle - M_PI + M_PI / 3) * arrowSize);
    
    // Draw the line itself
    //painter->setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter->setPen(QPen(Qt::black, 1, Qt::SolidLine));
    painter->drawLine(line);
    
    painter->setBrush(Qt::black);
    
    painter->drawPolygon(QPolygonF() << line.p2() << destArrowP1 << destArrowP2);
    //text
    painter->setPen(QPen(Qt::black, 0));
    //painter->drawText(boundingRect(), Qt::AlignCenter, instance);
    
}



//WIDGET:



GraphWidget::GraphWidget(QWidget *parent)
    : QGraphicsView(parent)
{
    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    //scene->setSceneRect(-200, -200, 400, 400);
    setScene(scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    scale(qreal(0.8), qreal(0.8));
    setMinimumSize(400, 400);
    setWindowTitle(tr("WM tree"));
    
    pthread_mutex_lock(&memMutex);
    
    centerNode = new Node(this, GUI_NODE_WMN);
    centerNode->loadWMN(WM);
    scene->addItem(centerNode);
    
    
    std::cout<<"GUI-tree: root loaded"<<std::endl;
    
    loadFromWM(scene,WM,centerNode);
    
    pthread_mutex_unlock(&memMutex);
    
    std::cout<<"GUI-tree: tree loaded"<<std::endl;
    
    //centerNode->setPos(-150,0);
    
    setPositionWMN(centerNode, -150, 0);
    
    setPositionCON();
    
    std::cout<<"GUI-tree: tree pos set"<<std::endl;
    
    setTransformationAnchor(QGraphicsView::NoAnchor);
}


void GraphWidget::loadFromWM(QGraphicsScene *scene, WM_node *node, Node *father_qt)
{
    int total_px_segment = 0;
    father_qt->depth = 0;
    
    //for each son node
    for(size_t i=0; i<node->son.size(); i++){
        //if the node is expanded and, if show-less-mode is enabled, the releaser is true
        if (node->son[i]->expanded ){ //&& ( node->son[i]->releaserStatus() ) ){
            
            //create a new agNode which represents it
            Node *node_qt = new Node(this, GUI_NODE_WMN);
            father_qt->addSon(scene, node_qt);
            node_qt->loadWMN(node->son[i]);
            
            showContributions(node_qt, node->son[i]);

            //recursively plot the subtree
            loadFromWM(scene,node->son[i],node_qt);
            
            total_px_segment += node_qt->px_segment;
            
            father_qt->depth = father_qt->depth > node_qt->depth + 1 ? father_qt->depth : node_qt->depth + 1;
        }
    }
    
    if(total_px_segment == 0) //I'm a leaf node
        father_qt->px_segment =  GUI_TREE_Y_STEP; //default px segment needed to visualize a single node
    else
        father_qt->px_segment = total_px_segment;
}

/*
 *  add to the graph possible contributors with their weights as a black box
 *  showing "ContributorName: Contribution x Weight"
 */
void GraphWidget::showContributions(Node *n, WM_node *node) {

    std::stringstream ss;
    bool first = true;
    
    if(node->contribution.empty())
        return;

    //set the node label

    // for each contribution
    for (auto it = node->contribution.begin(); it != node->contribution.end(); ++it) {
        //check if contribution is set
        if (it->second == NULL)
            continue;
        //get the associated weight
        double *w = WMV.get< std::unordered_map<std::string, double*> >(node->name + ".weights")[ instance2vector(it->first)[0] ];
        //check if the weight exist
        if (w == NULL)
            continue;
        
        if(!first)
            ss<<"\n";
        
        first = false;

        ss << it->first << ": " << roundec( *(it->second), 3 ) << "x" << roundec( *w, 3 );
        
    }

    //if all contributions have not been set, skip this node
    if(ss.str() == "")
        return;
    
    Node *cont_node = NULL;
    //int cont_i = 0;

    //search if the node exists
    for (size_t j = 0; j < contNodes.size(); j++) {
        if (contNodes[j]->target_node == n) {
            cont_node = contNodes[j];
            break;
        }
    }

    if (cont_node == NULL) {
        
        //std::cout << "\n showing " << ss.str() << std::endl;

        //create new node!
        cont_node = new Node(this, GUI_NODE_CON);

        cont_node->loadCON(ss.str(), n);
        //cont_node->instance = QString::fromStdString( ss.str() );

        contNodes.push_back(cont_node);

        this->scene()->addItem(cont_node);
        
        //create new edge

        Edge *e = new Edge(cont_node, n);
        this->scene()->addItem(e);
    }
    else {
        //update the node
        
        cont_node->loadCON(ss.str(), n);
    }

}

/*
//CHANGED 04/05/2023 to avoid crash on node removal
void GraphWidget::removeContributions(QGraphicsScene *scene, Node *node_qt)
{
    Node *con_node;
    
    int idx = -1;
    for(auto i=0; i<contNodes.size(); i++){
        if(contNodes[i]->target_node == node_qt){
            idx = i;
            con_node = contNodes[i];
        }
    }
    
    if(idx >= 0){
        
        contNodes.erase(contNodes.begin()+idx);
        
        Edge *old_edge;
        
        for(auto i=0; i<con_node->edges().size(); i++){
            if(con_node->edges()[i]->destNode() == node_qt){
                old_edge = con_node->edges()[i];
                con_node->edges().remove(i);
                break;
            }
        }
        
        scene->removeItem(con_node);
        scene->removeItem(old_edge);
        delete old_edge;
        delete con_node;
    }
}
*/
void GraphWidget::removeContributions(QGraphicsScene *scene, Node *node_qt)
{
    Node *con_node;
    
    size_t i = 0;
    while(i<contNodes.size()){
        if(contNodes[i]->target_node == node_qt){
            int idx = (int) i;
            con_node = contNodes[i];

            std::cout<<"GUI: erasing contribution " <<con_node->instance.toStdString()<<std::endl;

            contNodes.erase(contNodes.begin()+idx);
        
            Edge *old_edge;
            
            for(auto i=0; i<con_node->edges().size(); i++){
                if(con_node->edges()[i]->destNode() == node_qt){
                    old_edge = con_node->edges()[i];
                    con_node->edges().remove(i);
                    break;
                }
            }
            
            scene->removeItem(con_node);
            scene->removeItem(old_edge);
            delete old_edge;
            delete con_node;
        }
        else
            i++;
    }
}


////TODO
///*
// *  add to the graph contended variables
// *  showing "ContributorName: Contribution x Weight"
// */
//void GraphWidget::showVariables(Node *n, WM_node *node) {
//
//    std::stringstream ss;
//    bool first = true;
//    
//    if(node->contribution.empty())
//        return;
//
//    //set the node label
//
//    // for each contribution
//    for (auto it = WMV->begin(); it != wmv_end(); ++it) {
//        //check if contribution is set
//        if (it->second == NULL)
//            continue;
//        //get the associated weight
//        double *w = wmv_get< std::unordered_map<std::string, double*> >(node->name + ".weights")[it->first];
//        //check if the weight exist
//        if (w == NULL)
//            continue;
//        
//        if(!first)
//            ss<<"\n";
//        
//        first = false;
//
//        ss << it->first << ": " << roundec( *(it->second), 3 ) << "x" << roundec( *w, 3 );
//        
//    }
//    
//    Node *cont_node = NULL;
//    int cont_i = 0;
//
//    //search if the node exists
//    for (auto j = 0; j < contNodes.size(); j++) {
//        if (contNodes[j]->target_node == n) {
//            cont_node = contNodes[j];
//            break;
//        }
//    }
//
//    if (cont_node == NULL) {
//        
//        //std::cout << "\n showing " << ss.str() << std::endl;
//
//        //create new node!
//        cont_node = new Node(this, GUI_NODE_CON);
//
//        cont_node->loadCON(ss.str(), n);
//        //cont_node->instance = QString::fromStdString( ss.str() );
//
//        contNodes.push_back(cont_node);
//
//        this->scene()->addItem(cont_node);
//        
//        //create new edge
//
//        Edge *e = new Edge(cont_node, n);
//        this->scene()->addItem(e);
//    }
//    else {
//        //update the node
//        
//        cont_node->loadCON(ss.str(), n);
//    }
//
//}



void GraphWidget::removeSubtree(QGraphicsScene *scene, Node *root_qt)
{
    //for each son node
    //for(int i=0; i<root_qt->son.size(); i++){
    while(root_qt->son.size() > 0){

        std::cout<<"GUI: removing node " <<root_qt->instance.toStdString()<<std::endl;

        removeSubtree(scene,root_qt->son[root_qt->son.size()-1]);
        
        removeContributions(scene,root_qt->son[root_qt->son.size()-1]);
        
        root_qt->removeSon(scene,root_qt->son[root_qt->son.size()-1]);

        std::cout<<"GUI: node " <<root_qt->instance.toStdString()<<" removed"<<std::endl;
    }
}

void GraphWidget::setPositionWMN(Node *root_qt, int x, int y){
    
    root_qt->setPos(x,y);
    
    int seg_start = y + root_qt->px_segment/2;
    
    //std::cout<<"GUI-tree: setting "<<root_qt->instance.toStdString()<<std::endl;
    
    for(size_t i=0; i<root_qt->son.size(); i++){
        Node *son = root_qt->son[i];
        
        int x_pos = x + GUI_TREE_X_STEP;
        int y_pos = seg_start - son->px_segment/2;
        
        //std::cout<<"GUI-tree:\t setting "<<son->instance.toStdString()<< " to "<<x_pos<<", "<<y_pos<<std::endl;
        
        //son->setPos( x_pos, y_pos );
        
        setPositionWMN(son, x_pos, y_pos);
        
        son->px_target_pos_x = x_pos; //getPos have some delay
        son->px_target_pos_y = y_pos;
        
        seg_start -= son->px_segment;
    }
}

void GraphWidget::setPositionCON(){
    
    for(size_t i=0; i<contNodes.size(); i++){
        Node *n = contNodes[i];
        
        //int x_pos = n->target_node->getPos().x(); //- GUI_TREE_X_STEP/2;
        //int y_pos = n->target_node->getPos().y(); //- GUI_TREE_Y_STEP;
        
        int x_pos = n->target_node->px_target_pos_x - GUI_TREE_X_STEP/2;
        int y_pos = n->target_node->px_target_pos_y + GUI_TREE_Y_STEP/2;
        
        n->setPos( x_pos, y_pos );
        
        n->px_target_pos_x = x_pos; //getPos have some delay
        n->px_target_pos_y = y_pos;
    }
}

void GraphWidget::updateFromWM(QGraphicsScene *scene, WM_node *wm_node, Node *qt_node){
        
    //update the node
    qt_node->loadWMN(wm_node);
    qt_node->update();
    
    showContributions(qt_node, wm_node);

    //NOTE: wm_tree and qt_tree MUST be ordered in the same way!!!
    for (size_t i = 0; i < wm_node->son.size(); i++) {

        if( i >= qt_node->son.size() ){
            //the node is new ..add it to the tree

            //std::cout<<"GUI-tree: new node found "<<wm_node->son[i]->instance<<std::endl;
            
            //if the node is expanded and, if show-less-mode is enabled, the releaser is true
            if (wm_node->son[i]->expanded ){ //&& ( wm_node->son[i]->releaserStatus() ) ){
                //create a new agNode which represents it
                Node *new_qt_node = new Node(this, GUI_NODE_WMN);
                qt_node->addSon(scene, new_qt_node);
                new_qt_node->loadWMN(wm_node->son[i]);
                
                //recursively plot the subtree
                //updateFromWM(scene,wm_node->son[i],new_qt_node);
                loadFromWM(scene,wm_node->son[i],new_qt_node);
                
                qt_node->px_segment += new_qt_node->px_segment;
                //std::cout<<"GUI-tree: expanded! "<<std::endl;
                
                qt_node->depth = qt_node->depth > new_qt_node->depth + 1 ? qt_node->depth : new_qt_node->depth + 1;
            }
        }
        else{
            
            Node *current_son = qt_node->son[i];
            
            qt_node->px_segment -= current_son->px_segment;
            
            //update the node
            updateFromWM(scene, wm_node->son[i], current_son);
            
            qt_node->px_segment += current_son->px_segment; //it can be updated
            
            qt_node->depth = qt_node->depth > current_son->depth + 1 ? qt_node->depth : current_son->depth + 1;
        }
        
    }

    //remove qt_nodes if they are no more in WM
    //  04/05/2023 ADJUSTED TO SOLVE THE CRASH-ON-UPDATE PROBLEM
    //for (int i = wm_node->son.size(); i < qt_node->son.size(); i++) {
    size_t i=0;
    while (i<qt_node->son.size()){
        bool fnd = false;
        for(size_t j=0; j < wm_node->son.size(); j++) {
            if(qt_node->son[i]->instance.toStdString() == wm_node->son[j]->instance){
                fnd = true;
                break;
            }
        }
        if(!fnd){
            qt_node->px_segment -= qt_node->son[i]->px_segment;

            std::cout<<"GUI: detected node " <<qt_node->son[i]->instance.toStdString()<<" to be removed"<<std::endl;
            
            removeSubtree(scene, qt_node->son[i]);

            removeContributions(scene,qt_node->son[i]);

            std::cout<<"GUI: node " <<qt_node->son[i]->instance.toStdString()<<" removed"<<std::endl;

            qt_node->removeSon(scene,qt_node->son[i]);
        }
        else
            i++;
    }
}



void GraphWidget::itemMoved()
{
    if (!timerId)
        timerId = startTimer(1000 / 25);
}

void GraphWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
        centerNode->moveBy(0, -20);
        break;
    case Qt::Key_Down:
        centerNode->moveBy(0, 20);
        break;
    case Qt::Key_Left:
        centerNode->moveBy(-20, 0);
        break;
    case Qt::Key_Right:
        centerNode->moveBy(20, 0);
        break;
//    case Qt::Key_Plus:
//        zoomIn();
//        break;
//    case Qt::Key_Minus:
//        zoomOut();
//        break;
//    case Qt::Key_Space:
//    case Qt::Key_Enter:
//        shuffle();
//        break;
//    default:
//        QGraphicsView::keyPressEvent(event);
    }
}


void GraphWidget::timerEvent(QTimerEvent *event)
{
    Q_UNUSED(event);
    
    //UPDATE:
    pthread_mutex_lock(&memMutex);
    
    if(!dead())
        updateFromWM(scene(), WM, centerNode);
    
    //pthread_mutex_unlock(&memMutex);
    
    setPositionWMN(centerNode, -150, 0);
    
    setPositionCON();
     
    scene()->update(); //ERROR: crashed on forget -> SOLVED 04/05/2023

    pthread_mutex_unlock(&memMutex);
    

//    QVector<Node *> nodes;
//    const QList<QGraphicsItem *> items = scene()->items();
//    for (QGraphicsItem *item : items) {
//        if (Node *node = qgraphicsitem_cast<Node *>(item))
//            nodes << node;
//    }
//
//    for (Node *node : qAsConst(nodes))
//        node->calculateForces();
//
//    bool itemsMoved = false;
//    for (Node *node : qAsConst(nodes)) {
//        if (node->advancePosition())
//            itemsMoved = true;
//    }
//
//    if (!itemsMoved) {
//        killTimer(timerId);
//        timerId = 0;
//    }
}


void GraphWidget::wheelEvent(QWheelEvent *event)
{
    scaleView(pow(2., event->angleDelta().y() / 240.0));
}

void GraphWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        // Store original position.
        m_ori_pos = event->pos(); //mapToScene( event->pos() );
        
        //std::cout<<"start POS: "<<m_ori_pos.x()<<", "<<m_ori_pos.y()<<std::endl;
    }
}

void GraphWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (event->buttons() & Qt::LeftButton)
    {
//        QPointF oldp = mapToScene(m_originX, m_originY);
//        QPointF newp = mapToScene(event->pos());
        
        m_new_pos = event->pos(); //mapToScene( event->pos() );
        
        QPointF translation = m_new_pos - m_ori_pos;

        translate(translation.x(), translation.y());
        //this->horizontalScrollBar()->setValue( this->horizontalScrollBar()->value() + translation.x() );
        //this->verticalScrollBar()->setValue( this->verticalScrollBar()->value() + translation.y() );
        
        //std::cout<<"new POS: "<<m_new_pos.x()<<", "<<m_new_pos.y()<<std::endl;
        //std::cout<<"\ttranslate: "<<-translation.x()<<", "<<-translation.y()<<std::endl;
        
        m_ori_pos = m_new_pos;
    }
}

/*
void GraphWidget::drawBackground(QPainter *painter, const QRectF &rect)
{
    Q_UNUSED(rect);

    // Shadow
    //QRectF sceneRect = this->sceneRect();
    QRectF sceneRect = subtree_boundingRect( this->centerNode );
    
    QRectF rightShadow(sceneRect.right(), sceneRect.top() + 5, 5, sceneRect.height());
    QRectF bottomShadow(sceneRect.left() + 5, sceneRect.bottom(), sceneRect.width(), 5);
    if (rightShadow.intersects(rect) || rightShadow.contains(rect))
        painter->fillRect(rightShadow, Qt::darkGray);
    if (bottomShadow.intersects(rect) || bottomShadow.contains(rect))
        painter->fillRect(bottomShadow, Qt::darkGray);

    // Fill
    QLinearGradient gradient(sceneRect.topLeft(), sceneRect.bottomRight());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::lightGray);
    painter->fillRect(rect.intersected(sceneRect), gradient);
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(sceneRect);

    // Text
    QRectF textRect(sceneRect.left() + 4, sceneRect.top() + 4,
                    sceneRect.width() - 4, sceneRect.height() - 4);
//    QString message(tr("Click and drag the nodes around, and zoom with the mouse "
//                       "wheel or the '+' and '-' keys"));
    QString message(tr("Working Memory"));

    QFont font = painter->font();
    font.setBold(true);
    font.setPointSize(14);
    painter->setFont(font);
    painter->setPen(Qt::lightGray);
    painter->drawText(textRect.translated(2, 2), message);
    painter->setPen(Qt::black);
    painter->drawText(textRect, message);
}
*/

void GraphWidget::scaleView(qreal scaleFactor)
{
    qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
    if (factor < 0.07 || factor > 100)
        return;

    scale(scaleFactor, scaleFactor);
}

QRectF GraphWidget::subtree_boundingRect(Node *node_qt){
    
    //std::cout<<"target ps: "<<node_qt->px_target_pos_x<<std::endl;
    
    return QRectF(
            node_qt->px_target_pos_x - GUI_TREE_X_STEP/2 - node_qt->px_x_node/2,
            -node_qt->px_segment/2,
            node_qt->boundingRect().width() + ( node_qt->depth * ( GUI_TREE_X_STEP ) ),
            node_qt->px_segment
            );
}

