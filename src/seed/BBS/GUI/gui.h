/* 
 * File:   gui.h
 * Author: hargalaten
 *
 * Created on 23 luglio 2015, 12.58
 */

#ifndef GUI_H
#define	GUI_H

#include "seed.h"

#include "gui_tree.h"

#include <qt5/QtWidgets/QApplication>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QProgressBar>
#include <qt5/QtWidgets/QSlider>
#include <qt5/QtWidgets/QSplitter>

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

//#include <qt5/QtWidgets/QGraphicsWidget>
#include <QtCharts/QChart> //needs libqt5charts5-dev
#include <QtCharts/QLineSeries>
#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>

using namespace QtCharts;

class SeedWindow : public QMainWindow {
    //Q_OBJECT //seems to not work without qmake
public:
    //SeedWindow(QWidget *parent = 0) : QMainWindow(parent) {
    SeedWindow(QWidget *parent = 0) : QMainWindow(parent) {
        std::cout<<"in constructor"<<std::endl;
        frame = new QFrame(this);
        setCentralWidget(frame);
        
        QSplitter *v_splitter = new QSplitter(Qt::Vertical,frame);
        QSplitter *h_splitter = new QSplitter(Qt::Horizontal,frame);
        
        //QGridLayout *grid = new QGridLayout(frame);
        
        QWidget *widget_wm_graph = new GraphWidget(frame);
        QWidget *widget_wm = panel_wm();
        QWidget *widget_eg = panel_eg();
        QWidget *widget_send = panel_send();
        
        //grid->addWidget(widget_wm, 0, 0);
        //grid->addWidget(widget_eg, 1, 0);
        //grid->addWidget(widget_send, 2, 0, 1, 1);
        
        v_splitter->addWidget(widget_wm);
        v_splitter->addWidget(widget_eg);
        v_splitter->addWidget(widget_send);
        
        h_splitter->addWidget(v_splitter);
        h_splitter->addWidget(widget_wm_graph);
        
        QHBoxLayout *layout = new QHBoxLayout(frame);
        layout->addWidget(h_splitter);

        setWindowTitle(tr("Group Boxes"));
        //resize(860, 640);
        
        statusBar();
        
        //timerID = startTimer(100); //timer event every X ms
        timerID = startTimer(500);
        
//        timer = new QTimer(this);
//        timer->start(100);
        
    }
    ~SeedWindow(){
        std::cout<<"in destroyer"<<std::endl;
//        timer->stop();
//        delete timer;
        
        killTimer(timerID);
        
        
        delete frame;
        
        
        std::cout<<"GUI: destroyed"<<std::endl;
    }

private slots:

    void OnRunPressed() {
        
        pthread_mutex_lock(&memMutex);
        
        WM->getNodesByInstance("requestStream")[0]->addSon( commandEdit->text().toStdString() );
        
        pthread_mutex_unlock(&memMutex);
        
        statusBar()->showMessage(commandEdit->text() + " allocated", 2000);
        
    }

    void timerEvent(QTimerEvent *e) {
        
        pthread_mutex_lock(&memMutex);
        
        if(!dead())
            updateTree(WM,root);
        
        //updateBehaviorPanel();
        updateEg();
        
        pthread_mutex_unlock(&memMutex);

        Q_UNUSED(e);

    }
    
    void treeSelectionChanged(const QItemSelection & newSelection, const QItemSelection & oldSelection)
    {
        (void) newSelection; //unused
        (void) oldSelection; //unused

        //get the text of the selected item
        const QModelIndex index = tree->selectionModel()->currentIndex();
        QString selectedText = index.data(Qt::DisplayRole).toString();
        QString selectedHeadText = index.sibling(index.row(),0).data().toString();
        
        //find out the hierarchy level of the selected item
        int hierarchyLevel = 1;
        QModelIndex seekRoot = index;
        while (seekRoot.parent() != QModelIndex()) {
            seekRoot = seekRoot.parent();
            hierarchyLevel++;
        }
//        QString showString = QString("%1, Level %2").arg(selectedText)
//                                                    .arg(hierarchyLevel);
        
        QString showString = QString("%1, Level %2").arg(selectedHeadText)
                                                    .arg(hierarchyLevel);
        
//        //change behavior selection
//        behaviorEdit->setText( selectedText );
        
        //setWindowTitle(showString);
        statusBar()->showMessage(showString, 2000);
    }
    
    void openBehaviorTab(){
        
        const QModelIndex index = tree->selectionModel()->currentIndex();
        QString selectedBehaviorText = index.sibling(index.row(),0).data().toString();
        
        selected_path.clear();
        
        QModelIndex seekRoot = index;
        while (seekRoot.parent() != QModelIndex()) {
            
            selected_path.push_back( seekRoot.sibling(seekRoot.row(),0).data().toString().toStdString() );
            //std::cout<<"path: "<<seekRoot.sibling(seekRoot.row(),0).data().toString().toStdString()<<std::endl;
            
            seekRoot = seekRoot.parent();
        }
        
        pthread_mutex_lock(&memMutex);
        
        //std::cout<<"get node by path"<<std::endl;
        WM_node *selected_node = WM->getNodeByPath(selected_path);
        //std::cout<<"node is "<<selected_node->instance<<std::endl;
        
        //init
        if(selected_behavior == ""){
            behaviorText->setEnabled(true);
            behaviorTable->setEnabled(true);
            behaviorChartView->setEnabled(true);
            behaviorBtn->setEnabled(true);
            behaviorEdit->setEnabled(true);
        }
        
        selected_behavior = selectedBehaviorText.toStdString();
        
        behaviorSeries->clear();
        behaviorText->clear();
        behaviorTable->clear();
        
        behaviorChart->setTitle( QString::fromStdString("Emphasis (" + selected_behavior + ")") );
        
        QStringList m_TableHeader;
        m_TableHeader<<"Releaser"<<"Goal";
        behaviorTable->setVerticalHeaderLabels(m_TableHeader);
        
        QTableWidgetItem *ti;
        for(size_t i=0; i<selected_node->releaser.size(); i++){
            //std::cout<<"rel: "<<selected_node->releaser[i]<<std::endl;
            ti = new QTableWidgetItem();
            ti->setText( QString::fromStdString(selected_node->releaser[i]) );
            ti->setToolTip( ti->text() );
            
            if( (selected_node->releaser[i].at(0) != '-' && !WMV.get<bool>(selected_node->releaser[i])) ||
                (selected_node->releaser[i].at(0) == '-' && WMV.get<bool>(selected_node->releaser[i])) )
                ti->setForeground(QBrush(QColor(255, 0, 0)));
            else
                ti->setForeground(QBrush(QColor(0, 255, 0)));  
            
            behaviorTable->setItem(0, i, ti);
        }
        
        for(size_t i=0; i<selected_node->goal.size(); i++){
            //std::cout<<"rel: "<<selected_node->releaser[i]<<std::endl;
            ti = new QTableWidgetItem();
            ti->setText( QString::fromStdString(selected_node->goal[i]) );
            ti->setToolTip( ti->text() );
            
            if( !(selected_node->goal[i].at(0) != '-' && !WMV.get<bool>(selected_node->goal[i])) &&
                !(selected_node->goal[i].at(0) == '-' && WMV.get<bool>(selected_node->goal[i])) )
                ti->setForeground(QBrush(QColor(0, 0, 255)));
            
            behaviorTable->setItem(1, i, ti);
        }
        
        pthread_mutex_unlock(&memMutex);
        
        statusBar()->showMessage(selectedBehaviorText + " info loaded!", 2000);
        
    }
    
    void sendToBehavior(){
        
        pthread_mutex_lock(&memMutex);
        
        std::cout<< "sending msg \"" << behaviorEdit->text().toStdString() << "\" to behavior "<< selected_behavior << std::endl;
        
        pthread_mutex_unlock(&memMutex);
        
    }
    
    void updateBehaviorPanel(){
        
        if(selected_behavior == "")
            return;
        
        WM_node *node = WM->getNodesByInstance(selected_behavior)[0];
        
        double y;
        
        if(node->expanded)
            y = WM->getInstanceEmphasis(selected_behavior);
        else
            y = 0;
        
        
        if(y< yMin || y > yMax){
            if(y < yMin)
                yMin = y;
            if(y> yMax)
                yMax = y;
            axisY->setRange(yMin-1, yMax+1);
        }

        if(behaviorSeries->count() > 10){
            for(int i=0; i<10; i++){
                behaviorSeries->replace(i, i, behaviorSeries->at(i+1).y());
            }
            behaviorSeries->replace(10, 10, y);
        }
        else
            behaviorSeries->append(behaviorSeries->count(), y );
        
        behaviorChartView->repaint();
        
    }
    
    void egColorChanged(const QString & color_name){
        if(color_name == "SELECT")
            return;
        
        QColor new_color(color_name);
        
        int eg_index = egList->currentRow();
        if(eg_index == -1){
            return;
        }
        
        egSeries[eg_index]->setColor(new_color);
        egList->item(eg_index)->setBackgroundColor(new_color);
        
        egColor->setStyleSheet(QString("QLabel {border-width: 1px; border-style: solid; border-radius: 0px; background: ") + color_name + QString("}"));
        
    }
    
    void egSelectionChanged(){
        
        int eg_index = egList->currentRow();
        
        if(eg_index == -1)
            return;
        
        egCombo->setCurrentIndex(0);
        
        QString color_code = egList->item(eg_index)->backgroundColor().name();
        
        egColor->setStyleSheet(QString("QLabel {border-width: 1px; border-style: solid; border-radius: 0px; background: ") + color_code + QString("}"));
    }
    
    void removeFromGraphPressed() {
        
        int eg_index = egList->currentRow();
        if(eg_index == -1){
            return;
        }

        egChart->removeSeries(egSeries[eg_index]);

        //egList->removeItemWidget(egList->currentItem());
        delete egList->takeItem(eg_index);

        eg_behaviors.erase(eg_behaviors.begin()+eg_index);
        egSeries.erase(egSeries.begin()+eg_index);
        
        if(eg_behaviors.size() == 0){
            egRemoveBtn->setEnabled(false);
            egRemoveAllBtn->setEnabled(false);
            egCombo->setEnabled(false);
        }
        
    }
    
    void removeAllFromGraphPressed() {
        egChart->removeAllSeries();

        egList->clear();

        eg_behaviors.clear();
        egSeries.clear();
        
        egRemoveBtn->setEnabled(false);
        egRemoveAllBtn->setEnabled(false);
        egCombo->setEnabled(false);
    }
    
    void addFromTreePressed() {
        
        egRemoveBtn->setEnabled(true);
        egRemoveAllBtn->setEnabled(true);
        egCombo->setEnabled(true);
        
        //get the text of the selected item
        const QModelIndex index = tree->selectionModel()->currentIndex();
        QString selectedHeadText = index.sibling(index.row(),0).data().toString();
        
        eg_behaviors.push_back( selectedHeadText.toStdString() );
        
        int idx = eg_behaviors.size()-1;
        
        //add a new series in the graph
        egSeries.push_back( new QLineSeries(frame) );
        egChart->addSeries(egSeries[idx]);
        
        egSeries[idx]->attachAxis(eg_axisY);
        
        egSeries[idx]->attachAxis(eg_axisX);
        //done
        
        //add a new row in the list
        QListWidgetItem *li = new QListWidgetItem(egList);
        li->setText( selectedHeadText );
        li->setToolTip( li->text() );
        li->setBackgroundColor(egSeries[idx]->color());
        
        egList->addItem(li);
        //done
        
        statusBar()->showMessage(selectedHeadText + " add to graph", 2000);
        
    }
    
    void updateEg(){
        
        if(eg_behaviors.size() == 0)
            return;
        
        for(size_t i=0; i<eg_behaviors.size(); i++){
        
            if(WM->getNodesByInstance(eg_behaviors[i]).size() == 0){
                std::cout<<"GUI: removing "<<eg_behaviors[i]<<" from EG"<<std::endl;
                
                egChart->removeSeries(egSeries[i]);

                delete egList->takeItem(i);

                eg_behaviors.erase(eg_behaviors.begin()+i);
                egSeries.erase(egSeries.begin()+i);
                
                continue;
            }
            
            WM_node *node = WM->getNodesByInstance(eg_behaviors[i])[0];
            

            double y;

            if(node->expanded)
                y = WM->getInstanceEmphasis(eg_behaviors[i]);
            else
                y = 0;


            if(y< eg_yMin || y > eg_yMax){
                if(y < eg_yMin)
                    eg_yMin = y;
                if(y> eg_yMax)
                    eg_yMax = y;
                eg_axisY->setRange(eg_yMin-1, eg_yMax+1);
            }
            
            //std::cout<<"serie-"<<i<<": "<<egSeries[i]->count()<<std::endl;
            
            if(egSeries[i]->count() <= 100){
                //std::cout<<"\t insert: "<<y<<" at "<<100-egSeries[i]->count()<<std::endl;
                egSeries[i]->append(egSeries[i]->count(), y );
            }

            for(int j=egSeries[i]->count(); j>0; j--){
                //std::cout<<"\t replace: "<<j<<" with "<<j+1<<std::endl;
                egSeries[i]->replace(j, j, egSeries[i]->at(j-1).y());
            }
            egSeries[i]->replace(0, 0, y);

        }
        
        egChartView->repaint();
        
    }
    
private:
    
    QGroupBox *panel_wm(){
        QGroupBox *groupBox = new QGroupBox(tr("&Working Memory (WM)"), frame);
        
        QHBoxLayout *hbox = new QHBoxLayout(groupBox);
        
        tree = new QTreeWidget(frame);
        tree->setColumnCount(5);
        
        QStringList ColumnNames;
        ColumnNames << "Working Memory Tree" << "Emphasis"<< "Frequency" << "Releaser"<< "Goal"<< "State" << "ID";

        tree->setHeaderLabels(ColumnNames);
        tree->setColumnWidth(0,200);
        //item->setIcon(0, QIcon("your icon path or file name "));
        
        tree->setStyleSheet(QString::fromStdString(
                            //add the links to the tree
                            "QTreeView::branch:has-siblings:!adjoins-item {border-image: url(" + SEED_HOME_PATH + "/src/behavior/GUI/icons/stylesheet-vline.png) 0;}"
                            "QTreeView::branch:has-siblings:adjoins-item {border-image: url(" + SEED_HOME_PATH + "/src/behavior/GUI/icons/stylesheet-branch-more.png) 0;}"
                            "QTreeView::branch:!has-children:!has-siblings:adjoins-item {border-image: url(" + SEED_HOME_PATH + "/src/behavior/GUI/icons/stylesheet-branch-end.png) 0;}"
                            "QTreeView::branch:has-children:!has-siblings:closed,"
                            "QTreeView::branch:closed:has-children:has-siblings {border-image: none;image: url(" + SEED_HOME_PATH + "/src/behavior/GUI/icons/stylesheet-branch-closed.png);}"
                            "QTreeView::branch:open:has-children:!has-siblings,"
                            "QTreeView::branch:open:has-children:has-siblings  {border-image: none;image: url(" + SEED_HOME_PATH + "/src/behavior/GUI/icons/stylesheet-branch-open.png);}" 
//                            //customize item selection
//                            "QTreeView::item:selected {background: #c5ebfb;}"
//                            "QTreeView::branch:selected {background: #c5ebfb;}"
                            ) );
        
        pthread_mutex_lock(&memMutex);
        wm2qtTree(WM);
        pthread_mutex_unlock(&memMutex);
        
        //tree->move(10,10);
        //tree->resize(100,200);
        
        tree->expandAll();
        
        tree->setExpandsOnDoubleClick(false);
        
        connect(tree->selectionModel(), &QItemSelectionModel::selectionChanged, this, &SeedWindow::treeSelectionChanged);
        //connect(tree, &QTreeWidget::doubleClicked, this, &SeedWindow::openBehaviorTab);
        connect(tree, &QTreeWidget::doubleClicked, this, &SeedWindow::addFromTreePressed);
        
        tree->setAlternatingRowColors(true);
        //tree->setStyleSheet("QTreeWidget::item:selected { background-color: blue; }"); 
        
        hbox->addWidget( tree );
        
        groupBox->setLayout(hbox);
        //groupBox->setMinimumSize(200, 300); //min dimensiono the whole panel
        
        
        return groupBox;
    }
    
    //for a generic node
    void wm2qtTree(WM_node *node, QTreeWidgetItem *father){
        
        //for each son node
        for(size_t i=0; i<node->son.size(); i++){
            //if the node is expanded and, if show-less-mode is enabled, the releaser is true
            if (node->son[i]->expanded ){ //&& ( node->son[i]->releaserStatus() ) ){
                //create a new agNode which represents it
                QTreeWidgetItem *child = wmNode2qtNode(node->son[i], father);
                father->addChild( child );
                child->setExpanded(true);
                
                //recursively plot the subtree
                wm2qtTree(node->son[i],child);
            }
        }
    }
    
    //for the root
    void wm2qtTree(WM_node *node){
        
        root = wmNode2qtNode(node, NULL);
        
        //for each son node
        for(size_t i=0; i<node->son.size(); i++){
            //if the node is expanded and, if show-less-mode is enabled, the releaser is true
            if (node->son[i]->expanded ){ //&& ( node->son[i]->releaserStatus() ) ){
                //create a new agNode which represents it
                QTreeWidgetItem *child = wmNode2qtNode(node->son[i], root);
                root->addChild( child );
                child->setExpanded(true);
                
                //recursively plot the subtree
                wm2qtTree(node->son[i],child);
            }
        }
    }
    
    QTreeWidgetItem *wmNode2qtNode(WM_node *node, QTreeWidgetItem *father){
        
        QTreeWidgetItem *item;
        if(father == NULL)
            item = new QTreeWidgetItem(tree);
        else
            item = new QTreeWidgetItem(father);
        
        fillqtNode(node,item);
        
        return item;
    }
    
    
    bool releaser2qstring(WM_node *node, QString *qs){
        bool nodeReleased = true;
        std::stringstream ss;
        //for each element of the releaser
        for(size_t i=0; i<node->releaser.size(); i++){
            //if the element is false plot it as red
            if( (node->releaser[i].at(0) != '-' && !WMV.get<bool>(node->releaser[i])) ||
                (node->releaser[i].at(0) == '-' && WMV.get<bool>(node->releaser[i])) ){
                nodeReleased = false;
                //ss<<"<font color=\"red\">"<<node->releaser[i]<<"</font>"; //html not enabled in qtree
                ss<<"~"<<node->releaser[i];
            }
            //otherwise (is true) plot it as green
            else
                //ss<<"<font color=\"green\">"<<node->releaser[i]<<"</font>";
                ss<<node->releaser[i];

            //if there are other elements left, add an end line
            if(i != node->releaser.size()-1)
                ss<<"  &&  ";
                //ss<<"\n";
                //ss<<"<br />";
        }

    //        //show also the weight as black number between brackets
    //        double *w = wmv_get< std::unordered_map<std::string, double*> >(node->name + ".weights")[node->father->name];
    //        if(w!=NULL){
    //            ss<<"<br /><font color=\"black\">("<<*w<<")</font>";
    //        }

        if(node->releaser.size() != 0)
            //item->setText(2, QString::fromStdString( ss.str() ) );
            *qs = QString::fromStdString( ss.str() );
        else
            //item->setText(2, QString( "TRUE" ) );
            *qs = "TRUE";

        return nodeReleased;
    }

    bool goal2qstring(WM_node *node, QString *qs){
        // adjusted in SEED 6.0
        bool nodeGoal = true;

        std::stringstream ss;
        //for each element of the releaser
        for(size_t i=0; i<node->goal.size(); i++){

            // added in SEED 6.0
            std::string goal_str = node->goal[i];
            char c;
            if(node->goal[i].at(0) == '-'){
                std::stringstream ss2(goal_str);
                //discard the negation
                ss2>>c>>goal_str;
            }

            //if the element is false plot it as black
            if( (node->goal[i].at(0) != '-' && !WMV.get<bool>(goal_str)) ||
                (node->goal[i].at(0) == '-' && WMV.get<bool>(goal_str)) ){
                //ss<<"<font color=\"black\">"<<node->releaser[i]<<"</font>"; //html not enabled in qtree
                ss<<"~"<<node->goal[i];
                nodeGoal = false;
            }
            //otherwise (is true) plot it as blue
            else{
                //ss<<"<font color=\"blue\">"<<node->releaser[i]<<"</font>";
                ss<<node->goal[i];
            }

            //if there are other elements left, add an end line
            if(i != node->goal.size()-1)
                ss<<"  &&  "; //adjusted in SEED 6.0 (goal's elements are in AND)
                //ss<<"\n";
                //ss<<"<br />";
        }

        if(node->goal.size() != 0){
            //item->setText(3, QString::fromStdString( ss.str() ) );
            *qs = QString::fromStdString( ss.str() );
        }
        else{
            //item->setText(3, QString( "N/A" ) );
            *qs = "N/A";
            nodeGoal = false; //false by default if there is no goal
        }

        return nodeGoal;
    }
    
    
    void fillqtNode(WM_node *node, QTreeWidgetItem *item){
        
        std::stringstream ss;
        //if the node is in background add a fence (#) at the begin
        if(node->background)
            ss<<"#";
        
        ss<<node->instance;
        
        item->setText(0, QString::fromStdString( ss.str() ) ); //instance
        
        QFont font = tree->headerItem()->font(0);
        if(node->abstract){
            font.setItalic(true);
            font.setWeight(12); //thin
            item->setFont(0,font);
        }
        
        if(!node->expanded)
            return;
            
        //add the emphasis
        item->setText(1, QString::number( WM->getInstanceEmphasis(node->instance) ) );

        //add the frequency
        if(node->abstract)
            item->setText(2, QString( "N/A" ) );
        else
            item->setText(2, QString::number( node->rtm ) );
        
        //add the releaser
        QString qs;
        bool released = releaser2qstring(node, &qs);
        item->setText(3, qs );
        
        if( released ){
            item->setText(5, QString( "enabled" ) );
            item->setTextColor(5, QColor(0,255,0));
            //item->setBackgroundColor(4, QColor(0,255,0));
            item->setIcon(0, QIcon( QString::fromStdString( SEED_HOME_PATH + "/src/behavior/GUI/icons/icon_enabled.png" ) ) );
        }
        else{
            item->setText(5, QString( "disabled" ) );
            item->setTextColor(5, QColor(255,0,0));
            //item->setBackgroundColor(4, QColor(255,0,0));
            item->setIcon(0, QIcon( QString::fromStdString( SEED_HOME_PATH + "/src/behavior/GUI/icons/icon_disabled.png" ) ) );
        }
        
        //add the goal
        bool accomplished = goal2qstring(node, &qs);
        item->setText(4, qs );
        
        if( accomplished ){
            item->setText(5, QString( "accomplished" ) );
            item->setTextColor(5, QColor(0,0,255));
            //item->setBackgroundColor(4, QColor(0,0,255));
            item->setIcon(0, QIcon( QString::fromStdString( SEED_HOME_PATH + "/src/behavior/GUI/icons/icon_accomplished.png" ) ) );
        }
        
        //add the id
        item->setText(6, QString::number(node->id) );
        
    }
    
    
    
    void updateTree(WM_node *wm_node, QTreeWidgetItem *qt_node){
        
        //update the node
        fillqtNode(wm_node,qt_node);
        
        //NOTE: wm_tree and qt_tree MUST be ordered in the same way!!!
        
        for (size_t i = 0; i < wm_node->son.size(); i++) {
            
            if( (int) i >= qt_node->childCount() ){
                //the node is new ..add it to the tree
                
                //if the node is expanded and, if show-less-mode is enabled, the releaser is true
                if (wm_node->son[i]->expanded ){ //&& ( wm_node->son[i]->releaserStatus() ) ){
                    //create a new agNode which represents it
                    QTreeWidgetItem *child = wmNode2qtNode(wm_node->son[i], qt_node);
                    qt_node->addChild( child );
                    child->setExpanded(true);

                    //recursively plot the subtree
                    wm2qtTree(wm_node->son[i],child);
                }
            }
            else
                //update the node
                updateTree(wm_node->son[i], qt_node->child(i));
                //fillqtNode( wm_node->son[i], qt_node->child(i) );
        }
        
        //remove qt_nodes if they are no more in WM
        for (int i = wm_node->son.size(); i < qt_node->childCount(); i++) {
            qt_node->removeChild(qt_node->child(i));
        }
    }
    
    QGroupBox *panel_behavior(){
        QGroupBox *groupBox = new QGroupBox(tr("&Behavior info"), frame);
        
        QGridLayout *grid = new QGridLayout(groupBox);
//        QHBoxLayout *grid = new QHBoxLayout;
        
        
        behaviorTable = new QTableWidget(frame);
        behaviorTable->setRowCount(2);
        behaviorTable->setColumnCount(10);
        
        behaviorTable->horizontalHeader()->setVisible(false);
        //behaviorTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        //behaviorTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        //behaviorTable->setSelectionMode(QAbstractItemView::SingleSelection);
        //behaviorTable->setShowGrid(false);
        //behaviorTable->setStyleSheet("QTableView {selection-background-color: red;}");
        behaviorTable->setMaximumHeight(76);
        behaviorTable->setMinimumHeight(76);
        
        
        behaviorSeries = new QLineSeries(frame); //QLineSeries();
        
        behaviorChart = new QChart();
        behaviorChart->addSeries(behaviorSeries);
        behaviorChart->legend()->hide();
        behaviorChart->setTitle("Emphasis");
        
        axisY = new QValueAxis(frame);//QValueAxis;
        axisY->setTickCount(3);
        axisY->setLabelFormat("%i");
        behaviorChart->addAxis(axisY, Qt::AlignLeft);
        behaviorSeries->attachAxis(axisY);
        
        axisX = new QValueAxis(frame);//QValueAxis;
        axisX->setTickCount(5);
        axisX->setLabelFormat("%i");
        behaviorChart->addAxis(axisX, Qt::AlignBottom);
        behaviorSeries->attachAxis(axisX);
        
        yMax = 2;
        yMin = 0;
        
        axisX->setRange(0, 10);
        axisY->setRange(yMin, yMax);
        
        behaviorChartView = new QChartView(behaviorChart);
        behaviorChartView->setRenderHint(QPainter::Antialiasing);
        behaviorChartView->setMinimumSize(150,70); //behaviorChartView->setMinimumSize(300,150);
        behaviorChartView->setMaximumHeight(200);

        
        
        behaviorEdit = new QLineEdit(frame);
        connect(behaviorEdit, &QLineEdit::returnPressed, this, &SeedWindow::sendToBehavior);
        behaviorEdit->setToolTip("Write or Select a Behavior");
        behaviorEdit->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);
        behaviorEdit->setMinimumWidth(200);
//        behaviorEdit->setMaximumWidth(2000);
        
        behaviorBtn = new QPushButton("send", frame);
        connect(behaviorBtn, &QPushButton::clicked, this, &SeedWindow::sendToBehavior);
        behaviorBtn->setToolTip("send a command to this specific behavior");
        behaviorBtn->setMaximumSize(100,50);
        
        //QTextEdit *behaviorText = new QTextEdit(frame);
        behaviorText = new QTextEdit(frame);
        behaviorText->setReadOnly(true);
        //behaviorOut->setGeometry(0,0,200,300);
        behaviorText->append("a line");
        behaviorText->append("a new line");
        behaviorText->setMinimumSize(150,70); //behaviorText->setMinimumSize(300,150);
        behaviorText->setMaximumWidth(500);
        
//        grid->addWidget(behaviorRel, 0, 0);
//        grid->addWidget(behaviorGoal, 0, 1);
        grid->addWidget(behaviorTable, 0, 0, 1, 2);
        
        grid->addWidget(behaviorChartView, 1, 0, 1, 2);
        
        grid->addWidget(behaviorText, 2, 0, 1, 2);
        grid->addWidget(behaviorEdit, 3, 0);
        grid->addWidget(behaviorBtn, 3, 1);
        
        
        behaviorText->setEnabled(false);
        behaviorTable->setEnabled(false);
        behaviorChartView->setEnabled(true);
        behaviorBtn->setEnabled(false);
        behaviorEdit->setEnabled(false);
        
        
        selected_behavior = "";
        
        groupBox->setLayout(grid);
        
        return groupBox;
    }
    
    
    QGroupBox *panel_eg(){
        QGroupBox *groupBox = new QGroupBox(tr("&Emphasigraphy (EG)"), frame);
        
        QGridLayout *grid = new QGridLayout(groupBox);
        
        egList = new QListWidget(frame);
        connect(egList, &QListWidget::currentItemChanged, this, &SeedWindow::egSelectionChanged);
        egList->setMinimumHeight(100);
        egList->setMaximumWidth(200);
        
        egChart = new QChart();
        egChart->legend()->hide();
        egChart->setTitle("Emphasis");
        
        eg_axisY = new QValueAxis(frame);//QValueAxis;
        eg_axisY->setTickCount(3);
        eg_axisY->setLabelFormat("%i");
        
        eg_axisX = new QValueAxis(frame);//QValueAxis;
        eg_axisX->setTickCount(5);
        eg_axisX->setLabelFormat("-%i");
        
        egChart->addAxis(eg_axisY, Qt::AlignLeft);
        egChart->addAxis(eg_axisX, Qt::AlignBottom);
        
        eg_yMax = 2;
        eg_yMin = 0;
        
        eg_axisX->setRange(0, 100);
        eg_axisY->setRange(eg_yMin, eg_yMax);
        
        egChartView = new QChartView(egChart);
        egChartView->setRenderHint(QPainter::Antialiasing);
        egChartView->setMinimumSize(250,150); //egChartView->setMinimumSize(500,300);
        //egChartView->setMaximumHeight(200);
        
        
        QVBoxLayout *vbox = new QVBoxLayout();
//        vbox->setSpacing(10);
        
        QLabel *label1 = new QLabel("Setup Elements", this);
        
        egFromTreeBtn = new QPushButton("Add", frame);
        connect(egFromTreeBtn, &QPushButton::clicked, this, &SeedWindow::addFromTreePressed);
        egFromTreeBtn->setToolTip("add a new element to the graph from the tree (upper)");
        egFromTreeBtn->setMaximumSize(150,28);
        egFromTreeBtn->setMinimumSize(100,28);
        
        egRemoveBtn = new QPushButton("Remove", frame);
        connect(egRemoveBtn, &QPushButton::clicked, this, &SeedWindow::removeFromGraphPressed);
        egRemoveBtn->setToolTip("Remove the selected element from the EG");
        egRemoveBtn->setMaximumSize(150,28);
        egRemoveBtn->setMinimumSize(100,28);
        egRemoveBtn->setEnabled(false);
        
        egRemoveAllBtn = new QPushButton("Remove All", frame);
        connect(egRemoveAllBtn, &QPushButton::clicked, this, &SeedWindow::removeAllFromGraphPressed);
        egRemoveAllBtn->setToolTip("Remove all elements from the EG");
        egRemoveAllBtn->setMaximumSize(150,28);
        egRemoveAllBtn->setMinimumSize(100,28);
        egRemoveAllBtn->setEnabled(false);
        
        QLabel *label2 = new QLabel("Setup Colors", this);
        
        egCombo = new QComboBox(frame);
        setup_color_combo( egCombo );
        egCombo->setToolTip("Change the color in the graph");
        connect(egCombo, &QComboBox::currentTextChanged, this, &SeedWindow::egColorChanged);
        //connect(egCombo, &QComboBox::activated, this, &SeedWindow::egColorChanged);
        egCombo->setCurrentIndex(0);
        egCombo->setMaximumSize(120,28);
        egCombo->setMinimumSize(70,28);
        egCombo->setEnabled(false);
        
        egColor = new QLabel(this);
        egColor->setStyleSheet("QLabel {border-width: 1px; border-style: solid; border-radius: 0px;}"); // background: red}");
        egColor->setMaximumSize(28,28);
        egColor->setMinimumSize(28,28);
        
        QHBoxLayout *hbox_color = new QHBoxLayout();
        hbox_color->addWidget(egCombo);
        hbox_color->addWidget(egColor);
        
        vbox->setSpacing(3);
        vbox->addStretch(1);
        vbox->addWidget(label1);
        vbox->addWidget(egFromTreeBtn);
        vbox->addWidget(egRemoveBtn);
        vbox->addWidget(egRemoveAllBtn);
//        vbox->setSpacing(5);
        vbox->addWidget(label2);
        vbox->addLayout(hbox_color);
//        vbox->addWidget(egCombo);
//        vbox->addWidget(egColor);
        vbox->addStretch(1);
        
        //grid->addWidget(egFromTreeBtn, 0, 0, 1, 1, Qt::AlignCenter);
        grid->addWidget(egList, 0, 0, 3, 1);
        
        grid->addLayout(vbox, 0, 1, 3, 1);
        
        grid->addWidget(egChartView, 0, 2, 3, 1);
        
        
        
        groupBox->setLayout(grid);
        
        return groupBox;
    }
    
    
    void setup_color_combo(QComboBox *combo){
        
        const QStringList colorNames = QColor::colorNames();
        
        combo->insertItem(0, "SELECT");

        for (int i = 0; i < colorNames.size(); ++i) {
            QColor color(colorNames[i]);

            combo->insertItem(i+1, colorNames[i]);
            combo->setItemData(i+1, color, Qt::DecorationRole);
        }
        
    }
    
    
    QGroupBox *panel_send(){
        QGroupBox *groupBox = new QGroupBox(tr("&Seed Commands"), frame);
        
        //QHBoxLayout *hbox = new QHBoxLayout; //(frame);
        QGridLayout *grid = new QGridLayout(groupBox);
        
        commandEdit = new QLineEdit(frame);
        connect(commandEdit, &QLineEdit::returnPressed, this, &SeedWindow::OnRunPressed);
        commandEdit->setToolTip("Write an instance of Behavior");
        commandEdit->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);
//        commandEdit->setMinimumWidth(200);
//        commandEdit->setMaximumWidth(2000);
//        commandEdit->setGeometry(0,0,200,50);

        runBtn = new QPushButton("Send", frame);
        connect(runBtn, &QPushButton::clicked, this, &SeedWindow::OnRunPressed);
        runBtn->setToolTip("Insert the command in WM");
        runBtn->setMaximumSize(100,50);
        
        //hbox->addWidget(commandEdit, 10, Qt::AlignLeft | Qt::AlignBottom);
        //hbox->addWidget(runBtn, 0, Qt::AlignRight | Qt::AlignBottom);
        
        grid->addWidget(commandEdit,0,0);
        grid->addWidget(runBtn,0,1);
        
        //grid->addStretch(1);
        groupBox->setLayout(grid);
        groupBox->setFixedHeight(70);

        return groupBox;
    }

    QFrame *frame;
    
    //QTimer *timer;
    int timerID;
    
    QTreeWidget *tree;
    QTreeWidgetItem *root;
    
    QPushButton *runBtn;
    QLineEdit *commandEdit;
    
    std::string selected_behavior;
    std::vector<std::string> selected_path;
    
    QLineEdit *behaviorEdit;
    QPushButton *behaviorBtn;
    QTextEdit *behaviorText;
    QTableWidget *behaviorTable;
    
    //chart variables:
    QLineSeries *behaviorSeries;
    QChart *behaviorChart;
    QChartView *behaviorChartView;
    
    //std::vector< struct WM_graph > history;
    
    QValueAxis *axisX;
    QValueAxis *axisY;
    double yMax;
    double yMin;
    //--
    
    
    //emphasis graph
    QListWidget *egList;
    QPushButton *egFromTreeBtn;
    QComboBox *egCombo;
    QPushButton *egRemoveBtn;
    QPushButton *egRemoveAllBtn;
    
    QLabel *egColor;
    
    std::vector< std::string > eg_behaviors;
    
    //chart variables:
    QChart *egChart;
    QChartView *egChartView;
    
    std::vector< QLineSeries* > egSeries;
    QValueAxis* eg_axisX;
    QValueAxis* eg_axisY;
    double eg_yMax;
    double eg_yMin;
    //--
    
    
    //std::unordered_map< std::vector< double > > emph_map;
    
    //QWidget *behaviorText;
//    QTabWidget *behaviorTab; //this one crashes in construction
};


bool qt_application_running = false;


class GuiBehavior : public Behavior{
public:
    GuiBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    std::string getName(){
        return "gui";
    }
    bool perceptualSchema(){
        
        return true;
    }
    void motorSchema(){
        
        int argc;
        char **argv;

        qt_gui_application = new QApplication(argc, argv);
        std::cout<<"QT started"<<std::endl;
        
        //SeedWindow window;
        SeedWindow *window = new SeedWindow; //this must be a pointer

        std::stringstream ss;
        ss<<SEED_NAME<<" GUI";
        
        window->resize(500, 500);
        window->setWindowTitle(ss.str().c_str());
        window->show();
        
        
        qt_gui_application->exec();
        
//        delete window;
        
        QApplication::quit();
        
        std::cout<<"GUI: exited"<<std::endl;
        
        remove_from_WM(); //this code MUST be inside a function!
    }
    void start(){
    }
    void exit(){
//        pthread_mutex_lock(&memMutex);
//        if(dead())
//            return;
//        remove(WM->getNodesByInstance("alive")[0]);
//        pthread_mutex_unlock(&memMutex);
    }
    void remove_from_WM(){
        pthread_mutex_lock(&memMutex);
        if(dead())
            return;
        remove(WM->getNodesByInstance(getInstance())[0]);
        pthread_mutex_unlock(&memMutex);
    }
    
private:
    QApplication *qt_gui_application;
};


#endif	/* GUI_H */

