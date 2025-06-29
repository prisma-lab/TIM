/* 
 * File:   show.h
 * Author: hargalaten
 *
 * Created on 17 aprile 2015, 10.46
 */

#ifndef SHOW_H
#define	SHOW_H

#include "seed_header.h"
#include <graphviz/gvc.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// set to 1/0 to turn on/off the image visualization with opencv window
#define SHOW_OPENCV_IMAGE 0

/*
 *  show(nodeInstance,+mode): plots the tree rooted in the target node as a graph 
 *                  using the graphviz library. The image is also published in 
 *                  ROS through the /image_show topic.
 * 
 *      @nodeInstance: the instance of the root node.
 *      @mode: mode to plot the graph, if not specified the graph is fully plotted
 */
class ShowBehavior : public Behavior{
public:
    ShowBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        char * args[] = {
            (char *) "dot",
            (char *) "-Tpng", /*gif output*/
            (char *) "-Oabc1.png" /*output to file abc.gif*/
        };
        
        //get the list of nodes to be plotted
        targetNodes=WM->getNodesByInstance(instance2vector(instance)[1]);
        
        showLess = false;
        //if node is specified plot only active nodes!!
        if(instance2vector(instance).size() > 2)
            showLess = true;
        
        /* set up a graphviz context */
        gvc = gvContext();
        
        /* parse command line args - minimally argv[0] sets layout engine */
        gvParseArgs(gvc, sizeof(args)/sizeof(char*), args);
        
        //enable ROS image-stream
        //nh = rclcpp::Node::make_shared("solve");
        //ex.add_node(nh); // does it work? NO!

        //image_transport::ImageTransport it(nh);
        //image_pub = it.advertise(SEED_NAME + "/show", 1);

        image_pub = image_transport::create_publisher(nh.get(), SEED_NAME + "/show");
        //image_pub = nh->create_publisher<sensor_msgs::msg::Image>(SEED_NAME + "/show", 1);
        
    }
    std::string getName(){
        return "show";
    }
    bool perceptualSchema(){
        this->setRate(1); // one execution per second
        return true;
    }
    void motorSchema(){
        
        bool targetNotAlive=false;
        /* Create a simple digraph */
        graph = agopen((char *) "g", Agdirected,0);
        
        pthread_mutex_lock(&memMutex);
        if(dead())
            return;
        //check if the root node is deallocated during the sleeping time
        std::vector<WM_node*> meVec = WM->getNodesByInstance(this->getInstance());
        if (meVec.size() == 0) {
            std::cout << this->getInstance() << " (ms) DEALLOCATED\n";
            pthread_mutex_unlock(&memMutex);
            return;
        }

        //do not amplify this process
        //WM_node* me = meVec[0];
        //me->amplification=0;
        
        //setRtm(0.1);
        Agnode_t * alive;
        //if the target node is not alive
        if(targetNodes.size()!=0 && targetNodes[0]!=WM){
            targetNotAlive=true;
            //allocate the agNode of alive as the root of the graph 
            alive=wmNode2agNode(WM);
        }
        //for each wm-node of the target instance
        for (size_t i = 0; i < targetNodes.size(); i++) {
            //create a new agNode as the root of the sub-tree
            Agnode_t *root = wmNode2agNode(targetNodes[i]);
            //if the target node is not alive
            if(targetNotAlive){
                //add an edge connecting the new node with the alive one
                Agedge_t *e = agedge(graph, alive, root, 0, 1);
                //this edge is dashed (summarizes the path for alive)
                agsafeset(e,(char *)"style",(char *)"dashed",(char *)"");
            }
            //recursively build the graph
            wm2graph(targetNodes[i], root);
        }
        pthread_mutex_unlock(&memMutex);
        
        /* Compute a layout using layout engine from command line args */
        gvLayoutJobs(gvc, graph);
        /* Write the graph according to -T and -o options */
        gvRenderJobs(gvc, graph);
        /* Free layout data */
        gvFreeLayout(gvc, graph);
        /* Free graph structures */
        agclose(graph);
        
        cv::Mat graphImg;
        //read the png image generated form graphviz
        //graphImg=cv::imread("noname.gv.png",cv::CV_LOAD_IMAGE_COLOR);
        graphImg=cv::imread("noname.gv.png");
        if (!graphImg.data) // Check for invalid input
        {
            std::cout << "Could not open or find the image" << std::endl;
            return;
        }

        //publish image
        //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", graphImg).toImageMsg();
        //image_pub.publish(msg);
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", graphImg).toImageMsg();
        image_pub.publish(msg);


#if SHOW_OPENCV_IMAGE
        cv::imshow(this->getInstance(), graphImg); // Show our image inside it.
        cv::waitKey(3);
#endif
        
    }
    /*
     *  this function transform a wm-tree, rooted in @node into 
     *  a graphviz graph (to be visualized as image).
     */
    void wm2graph(WM_node *node, Agnode_t *f){
        
        //for each son node
        for(size_t i=0; i<node->son.size(); i++){
            //if the node is expanded and, if show-less-mode is enabled, the releaser is true
            if (node->son[i]->expanded && ( !showLess || node->son[i]->releaserStatus() ) ){
                //create a new agNode which represents it
                Agnode_t *n = wmNode2agNode(node->son[i]);
                
                //add an edge between the father and the new agnode 
                Agedge_t *e = agedge(graph, f, n, 0, 1);
                //plot the realeaser on the edge
                showReleaserOnEdge(e,node->son[i]);
                
                //recursively plot the subtree
                wm2graph(node->son[i], n);
            }
        }
        
    }
    
    /*
     *  transform the wm-node @node into an agnode (returned) 
     */
    Agnode_t *wmNode2agNode(WM_node* node){
        std::stringstream ss;
        //if the node is in background add a fence (#) at the begin
        if(node->background)
            ss<<"#";

        //truncate too long strings
        std::string str_inst = node->instance;

        // 50 chars limit for the instance
        str_inst = shortenString(str_inst,50);
        
        //create a string containing the instance and the emphasis
        ss << str_inst << "\n" 
                << "(" << WM->getInstanceEmphasis(node->instance) << ")"; 
        
        std::string str = ss.str();
        //transform the string in char*
        char * writable = new char[str.size() + 1];
        std::copy(str.begin(), str.end(), writable);
        writable[str.size()] = '\0';
        
        //create a new agnode
        Agnode_t *n = agnode(graph, writable, 1);
        
        //show possible contributions (additional black nodes)
        showContributions(n,node);
        
        //set the node shape
        delete[] writable;
        agsafeset(n, (char *)"penwidth", (char *)"2.0", (char *)"");
        if(node->abstract)
            agsafeset(n, (char *)"style", (char *)"dashed", (char *)"");
        
        //if goal is reached
        if(node->goalStatus())
            //node color is blue
            agsafeset(n, (char *)"color", (char *)"blue", (char *)"");
        //if node is activated (ie. not quiescent and released)
        else if (node->rtm != QUIESCENCE && WM->isReleased(node->instance)) { //node->son[i]->isBranchReleased() ){
            //node color is green
            agsafeset(n, (char *)"color", (char *)"green", (char *)"");
        } 
        //otherwise (deactivated)
        else 
            //if the color is not already green (ie. no activated instance of this node was already detected)
            if (agget(n, (char *)"color") != (char *)"green")
                //node color is red
                agsafeset(n, (char *)"color", (char *)"red", (char *)"");
        
        //return the new node
        return n;
    }
    
    /*
     *  add to the graph possible contributors with their weights as a black box
     *  showing "ContributorName: Contribution x Weight"
     */
    void showContributions(Agnode_t *n, WM_node *node){
        
        std::stringstream ss;
        bool first = true;
        
       // for each contribution
        for (auto it = node->contribution.begin(); it != node->contribution.end(); ++it) {
            //check if contribution is set
            if(it->second == NULL)
                continue;
            //get the associated weight
            double *w = wmv_get< std::unordered_map<std::string, double*> >(node->name + ".weights")[it->first];
            //check if the weight exist
            if(w == NULL)
                continue;
            
            //if not the first, add an end line
            if(first)
                first = false;
            else
                ss<<"\n";

            //set the node label
            ss<<it->first<<": "<<*(it->second)<<"x"<< *w;
        }
        
        //if not first means that at least one contributor is found
        if(!first){
            std::string str = ss.str();
            //transform the string in char*
            char * writable = new char[str.size() + 1];
            std::copy(str.begin(), str.end(), writable);
            writable[str.size()] = '\0';
            //create a new agnode
            Agnode_t *n_cont = agnode(graph, writable, 1);
            delete[] writable;

            //set box shape
            agsafeset(n_cont, (char *)"shape", (char *)"box", (char *)"");
            agsafeset(n_cont, (char *)"fontsize", (char *)"10", (char *)"");
            agsafeset(n_cont, (char *)"height", (char *)"0.25", (char *)"");

            //create a new black edge connecting the two nodes
            agedge(graph, n_cont, n, 0, 1); //Agedge_t *e = agedge(graph, n_cont, n, 0, 1);
        }
        
    }
    
    /*
     *  plot the elements of the releaser of @node over the edge @e
     */
    void showReleaserOnEdge(Agedge_t *e, WM_node *node){
        bool nodeReleased = true;
        std::stringstream ss;
        //for each element of the releaser
        for(size_t i=0; i<node->releaser.size(); i++){
            //if the element is false plot it as red
            if( (node->releaser[i].at(0) != '-' && !wmv_get<bool>(node->releaser[i])) ||
                (node->releaser[i].at(0) == '-' && wmv_get<bool>(node->releaser[i])) ){
                nodeReleased = false;
                //ss<<"~";
                ss<<"<font color=\"red\">"<<node->releaser[i]<<"</font>";
            }
            //otherwise (is true) plot it as green
            else
                ss<<"<font color=\"green\">"<<node->releaser[i]<<"</font>";

            //if there are other elements left, add an end line
            if(i != node->releaser.size()-1)
                //ss<<"\n";
                ss<<"<br />";
        }
        
        //show also the weight as black number between brackets
        double *w = wmv_get< std::unordered_map<std::string, double*> >(node->name + ".weights")[node->father->name];
        if(w!=NULL){
            ss<<"<br /><font color=\"black\">("<<*w<<")</font>";
        }

        std::string str = ss.str();
        //transform the string in char*
        char * writable = new char[str.size() + 1];
        std::copy(str.begin(), str.end(), writable);
        writable[str.size()] = '\0';
        //interpret the html tags
        char *lab = agstrdup_html(agroot(e),writable);

        //set the label
        agsafeset(e,(char *)"label", lab, (char *)"");
        delete[] writable;
        
        //string must be removed!!
        agstrfree(agroot(e),lab);
        
        if(nodeReleased)
            agsafeset(e, (char *)"color", (char *)"green", (char *)""); //agsafeset(e,"fontcolor","green","");
        else
            agsafeset(e, (char *)"color", (char *)"red", (char *)""); //agsafeset(e,"fontcolor","red","");
    }

    // cut string lenght if it is too long
    std::string shortenString(const std::string& str, size_t k) {
        if (str.length() <= k) return str;

        const std::string ellipsis = "...";
        if (k <= ellipsis.length() + 1) return str.substr(0, k);  // Can't shorten meaningfully

        size_t remaining = k - ellipsis.length();
        size_t prefixLength = remaining / 2;
        size_t suffixLength = remaining - prefixLength;

        return str.substr(0, prefixLength) + ellipsis + str.substr(str.length() - suffixLength);
    }
    
    void start() {

    }
    void exit(){
        //free the graph context
        gvFreeContext(gvc);
#if SHOW_OPENCV_IMAGE
        // close window
        cv::destroyWindow(this->getInstance());
        cv::waitKey(1);
#endif
    }
protected:
    GVC_t *gvc;
    Agraph_t *graph;
    std::vector< WM_node * > targetNodes;
    
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    image_transport::Publisher image_pub;
    
    bool showLess;
};


#endif	/* SHOW_H */

