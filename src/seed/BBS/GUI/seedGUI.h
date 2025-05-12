/* 
 * File:   seedGUI.h
 * Author: hargalaten
 *
 * Created on 29 maggio 2016, 10.27
 */
#include "../seed_header.h"
#include "../Fg.h"
//#include <gtk-3.0/gtk/gtk.h>
#include <gtk-2.0/gtk/gtk.h>

class WM_behavior_graph{
public:
    WM_behavior_graph(std::string target, GtkBuilder *gtkBuilder){
        
        //set static colors
        std::vector< cv::Scalar > staticColor;
        staticColor.push_back( cv::Scalar(0,255,0) );
        staticColor.push_back( cv::Scalar(0,0,255) );
        staticColor.push_back( cv::Scalar(0,215,255) );
        staticColor.push_back( cv::Scalar(0,128,255) );
        staticColor.push_back( cv::Scalar(255,0,0) );
        
        schema = target;//instance2vector(clear)[i+1];
        subSchema = "";
        //app.color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 200));
        color=staticColor[4];
        init(value);
        magnitude=0;
        
        oldImage=NULL;
        frame=GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "graph_frame"));
        window = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "seed_behavior"));
        builder = gtkBuilder;
    }
    WM_behavior_graph(std::string target, GtkWidget *gtkWidget, GtkWidget *fgbox){
        
        //set static colors BGR
        std::vector< cv::Scalar > staticColor;
        staticColor.push_back( cv::Scalar(0,255,0) );
        staticColor.push_back( cv::Scalar(0,0,255) );
        staticColor.push_back( cv::Scalar(0,215,255) );
        staticColor.push_back( cv::Scalar(0,128,255) );
        staticColor.push_back( cv::Scalar(255,0,0) );
        
        schema = target;//instance2vector(clear)[i+1];
        subSchema = "";
        //app.color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 200));
        color=staticColor[4];
        init(value);
        magnitude=0;
        
        oldImage=NULL;
        frame = gtkWidget;//GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "graph_frame"));
        window = fgbox;//GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "seed_behavior"));
        builder = gtk_builder_new();
    }
    WM_behavior_graph(){
        schema = "";//instance2vector(clear)[i+1];
        subSchema = "";
        //app.color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 200));
        color=cv::Scalar(0,255,0);
        init(value);
        magnitude=0;
        
        oldImage=NULL;
        frame=NULL;
        builder = NULL;
        window = NULL;
    }
    void shift(float head){
        for(int i=1;i<HISTORYTIME;i++){
            value[i-1]=value[i];
        }
        value[HISTORYTIME-1]=head;
    }
    void init(float *arr){
        for(int i=0;i<HISTORYTIME;i++){
            arr[i]=1;
        }
    }
    std::string schema;
    std::string subSchema;
    cv::Scalar color;
    float value [HISTORYTIME];
    double magnitude;
    
    GtkWidget *oldImage;
    GtkWidget *frame;
    GtkWidget *window;
    GtkBuilder *builder;
};

class guiBehavior : public Behavior{
public:
    guiBehavior(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        colorMap["blue"]  = cv::Scalar(255,0,0);
        colorMap["green"] = cv::Scalar(0,255,0);
        colorMap["red"]   = cv::Scalar(0,0,255);
        
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        
        /* All GTK applications must have a gtk_main(). Control ends here
         * and waits for an event to occur (like a key press or
         * mouse event). */
        gtk_main ();
        g_slice_free (GSList, wList);
    }
    static gboolean plot_fg(){
        
        pthread_mutex_lock(&memMutex);
        
        for(int i=0; i<history.size(); i++){
        
            std::string target = history[i].schema;
            
            //std::cout<<"updating " << target <<"..\n";

            double apprtm;
            
            if(dead())
                return FALSE;

            if(WM->getNodesByInstance(target).size()!=0){

                history[i].subSchema="";

                if(WM->getNodesByInstance(target)[0]->abstract){

                    std::vector< WM_node* > bestList;
                    for(int j=0; j<WM->getNodesByInstance(target).size(); j++){
                        bestList.push_back(sortNodes(WM->getNodesByInstance(target)[0]->tree2list())[0]);
                    }

                    history[i].subSchema=sortNodes(bestList)[0]->instance;

                    WM_node * pivot=getReleasedInstance(history[i].subSchema);

                    apprtm=pivot->rtm;
                    history[i].magnitude=WM->getInstanceEmphasis(history[i].subSchema);
                }
                else{

                    WM_node * pivot=getReleasedInstance(target);

                    apprtm=pivot->rtm;
                    history[i].magnitude=WM->getInstanceEmphasis(target);
                }


                if(apprtm == QUIESCENCE)
                    apprtm=1.1;

                history[i].shift(apprtm);

            }
            else{
                history[i].shift(1);
                history[i].magnitude=0;
            }
            
        }
        pthread_mutex_unlock(&memMutex);

        //scala l'altezza in base agli schemi da visualizzare (MASSIMO 5)
        int scaled_height= 140; //+ (110);
        int line_step = 5;
        int figure_width = HISTORYTIME * line_step + 60;
        int fg_width = HISTORYTIME * line_step;
        
        //crea la matrice con l'altezza scalata
        cv::Mat image( scaled_height, figure_width, CV_8UC3, cv::Scalar(240, 240, 240));
        int floor=FG_HEIGHT-FG_BASE;
        //disegna il grafico complessivo di tutti gli schemi
        for(int i=0;i<history.size(); i++){
            for (int j=1; j < HISTORYTIME; j++) {
                cv::line(image,cv::Point(FG_START+((j-1)*line_step),floor-( (1-history[i].value[j-1])*100)),
                        cv::Point(FG_START+(j*line_step),floor-( (1-history[i].value[j])*100)), history[i].color,2,8,0 );
            }
        }
        //crea le linee orizzontali del grafico
        std::stringstream ss;
        for(int i=1;i<10;i++){
            cv::line(image,cv::Point(FG_START,floor-(i*10)),cv::Point(FG_START+fg_width,floor-(i*10)), cv::Scalar(0,0,0),1,8,0);
            ss<<i*10;
            cv::putText(image,ss.str(),cv::Point(FG_START+fg_width+1,floor-(i*10)),cv::FONT_HERSHEY_SIMPLEX,0.3,cv::Scalar(0,0,0),1,8,false);
            ss.str("");
        }
        //crea la linea orizzontale di altezza 100
        cv::line(image, cv::Point(FG_START, floor - 100), cv::Point(FG_START + fg_width, floor - 100), cv::Scalar(0, 0, 0), 1, 8, 0);
        ss << 100;
        cv::putText(image, ss.str(), cv::Point(FG_START + fg_width + 1, floor - 100), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
        ss.str("");
        //crea la linea orizzontale di altezza 0
        cv::line(image, cv::Point(FG_START, floor), cv::Point(FG_START + fg_width, floor), cv::Scalar(0, 0, 0), 1, 8, 0);
        ss << 0;
        cv::putText(image, ss.str(), cv::Point(FG_START + fg_width + 1, floor), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
        ss.str("");
        //crea la linea orizzontale di altezza -1 (quiescenza)
        cv::line(image,cv::Point(FG_START,floor+10),cv::Point(FG_START+fg_width,floor+10), cv::Scalar(0,0,0),1,8,0);
        cv::putText(image,"(q)",cv::Point(FG_START+fg_width+1,floor+10),cv::FONT_HERSHEY_SIMPLEX,0.3,cv::Scalar(0,0,0),1,8,false);

        cv::rectangle(image,cv::Point(FG_START,5),cv::Point(figure_width-10,floor+15), cv::Scalar(0,0,0),2,8,0);

        GtkWidget *gtkImg = mat2widget(image,figure_width,scaled_height);
        
        //std::cout<<"show-update\n";
        if(oldFg != NULL){
            //std::cout<<"oldImg removed\n";
            gtk_container_remove(GTK_CONTAINER (fg_total_view), oldFg);
        }
        gtk_container_add (GTK_CONTAINER (fg_total_view), gtkImg);
        //gtk_widget_set_size_request(fg_total_view,400,200);
        oldFg = gtkImg;
        gtk_widget_show (gtkImg);

        //disegna il grafico del singolo schema
        for(int i=0;i<history.size(); i++){
            
            cv::Mat image( 90, figure_width, CV_8UC3, cv::Scalar(240, 240, 240));
            
            int c_start = FG_START;
            int c_floor = 70;//floor+(FG_HEIGHT/2 ) ;
            
            //DISEGNA IL SOTTOGRAFICO
            
            //disegna la funzione nel grafico
            for (int j = 1; j < HISTORYTIME; j++) {
                cv::line(image, cv::Point(c_start + ((j - 1)*line_step), c_floor - ((1 - history[i].value[j - 1])*50)),
                        cv::Point(c_start + (j * line_step), c_floor - ((1 - history[i].value[j])*50)), history[i].color, 2, 8, 0);
            }

            std::stringstream ss;
            //disegna le linee parallele del grafico
            cv::line(image, cv::Point(c_start, c_floor), cv::Point(c_start + fg_width, c_floor), cv::Scalar(0, 0, 0), 1, 8, 0);
            ss << 0;
            cv::putText(image, ss.str(), cv::Point(c_start + fg_width + 1, c_floor), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
            ss.str("");

            cv::line(image, cv::Point(c_start, c_floor - 50), cv::Point(c_start + fg_width, c_floor - 50), cv::Scalar(0, 0, 0), 1, 8, 0);
            ss << 100;
            cv::putText(image, ss.str(), cv::Point(c_start + fg_width + 1, c_floor - 50), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
            ss.str("");

            cv::line(image, cv::Point(c_start, c_floor + 10), cv::Point(c_start + fg_width, c_floor + 10), cv::Scalar(0, 0, 0), 1, 8, 0);
            cv::putText(image, "(q)", cv::Point(c_start + fg_width + 1, c_floor + 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1, 8, false);
            
            cv::rectangle(image,cv::Point(c_start,c_floor - 60),cv::Point(figure_width-10,c_floor + 15), cv::Scalar(0,0,0),2,8,0);
            
            GtkWidget *gtkImg = mat2widget(image,figure_width,90);
            
            //std::cout<<"show-update\n";
            if(history[i].oldImage != NULL){
                //std::cout<<"oldImg removed\n";
                gtk_container_remove(GTK_CONTAINER (history[i].frame), history[i].oldImage);
            }
            gtk_container_add (GTK_CONTAINER (history[i].frame), gtkImg);
            gtk_widget_set_size_request(history[i].frame,400,100);
            history[i].oldImage = gtkImg;
            gtk_widget_show (gtkImg);
        }

        //std::cout<<"..updated\n";
        
        //std::cout<<"end-of-update\n";
        return TRUE;
    }
    static GtkWidget *mat2widget(cv::Mat image, int width, int height){
        GtkWidget* gtkImg = NULL;
        GdkPixbuf* gtkPixbuf = NULL;
        IplImage* dstImage = NULL;
        IplImage* srcImage = new IplImage(image);
        
        //Creating the destionation image
        dstImage = cvCreateImage( cvSize(width,height), IPL_DEPTH_8U, 3);

        // Converting the format of the picture from BGR to RGB
        cvCvtColor ( srcImage, dstImage, CV_BGR2RGB );

        // Creates a new GdkPixbuf out of in-memory image data
        gtkPixbuf = gdk_pixbuf_new_from_data ( (guchar*)dstImage->imageData,
        GDK_COLORSPACE_RGB,
        FALSE,
        dstImage->depth,
        dstImage->width,
        dstImage->height,
        (dstImage->widthStep),
        NULL,
        NULL
        );

        // Create new GtkImage displaying pixbuf
        gtkImg = gtk_image_new_from_pixbuf ( gtkPixbuf );
        g_object_unref (gtkPixbuf);
        
        return gtkImg;
    }
    static void load_fgbox_window(GtkWidget *button, GtkWidget *entry){
        //std::cout<<"OPEN NEW WINDOW\n";
        
        GtkBuilder * gtkBuilder_behavior;
        GtkWidget *window, *frame;
        
        const gchar* gtarget = gtk_entry_get_text(GTK_ENTRY(entry));
        std::string target(gtarget);
        
        gtkBuilder_behavior = gtk_builder_new();
        gtk_builder_add_from_file(gtkBuilder_behavior,"/home/hargalaten/glade_ws/seed_behavior.glade",NULL);
        
        window = GTK_WIDGET(gtk_builder_get_object(gtkBuilder_behavior, "seed_behavior"));
        gtk_window_set_title(GTK_WINDOW(window),gtarget);
        
        wList = g_slist_append(wList,window);
        
        history.push_back(WM_behavior_graph(target,gtkBuilder_behavior));
        
        //g_signal_connect(window, "delete-event", G_CALLBACK( delete_event_fg ), (gpointer) gtarget);
        g_signal_connect (G_OBJECT (window), "destroy", G_CALLBACK (destroy_fg), NULL); //, wList);
        
        gtk_widget_show(window);
        
        //g_timeout_add_seconds(1,(GSourceFunc)plot_fg, (gpointer) gtarget );
        //std::cout<<"DONE\n";
    }
    
    static void load_fgbox_widget(GtkWidget *button, GtkWidget *entry){
        //std::cout<<"OPEN NEW WIDGET\n";
        
        GtkWidget *widget, *fg_box, *new_fgbox, *new_inner_box, *new_lower_box, *new_upper_box;
        GtkWidget *view, *x_button, *frame;
        
        const gchar* gtarget = gtk_entry_get_text(GTK_ENTRY(entry));
        std::string target(gtarget);
        
        fg_box = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "fg_box"));
        
        new_fgbox = gtk_vbox_new(FALSE,0);
        
        new_upper_box = gtk_hbox_new(FALSE,0);
        
        x_button = gtk_button_new_with_label("x");
        gtk_widget_set_size_request(x_button,30,30);
        //gtk_button_set_relief(GTK_BUTTON(x_button),GTK_RELIEF_NONE);
        frame = gtk_frame_new(gtarget);
        
//        GdkColor *defaultColor;
//        gdk_color_parse("blue", defaultColor);
//        GtkWidget *col_button = gtk_color_button_new_with_color(defaultColor);
        
        GtkWidget *col_combo = gtk_combo_box_new_text();

        for (std::map<std::string, cv::Scalar>::iterator it = colorMap.begin(); it != colorMap.end(); ++it) {
            std::cout<<"cbox: "<<it->first.c_str()<<"\n";
            gtk_combo_box_append_text(GTK_COMBO_BOX(col_combo), it->first.c_str());
        }
        
        gtk_combo_box_set_active(GTK_COMBO_BOX(col_combo),0);
        gtk_widget_set_size_request(col_combo,100,30);
        
        gtk_box_pack_start(GTK_BOX(new_upper_box), x_button, FALSE, TRUE, 0 );
        gtk_box_pack_end(GTK_BOX(new_upper_box), col_combo, FALSE, TRUE, 0 );
        
        gtk_box_pack_start(GTK_BOX(new_fgbox), new_upper_box, FALSE, TRUE, 0 );
        
        view = gtk_viewport_new(NULL,NULL);
        
        gtk_box_pack_start(GTK_BOX(new_fgbox), view, FALSE, TRUE, 0 );
        gtk_container_add(GTK_CONTAINER(frame), new_fgbox);
        gtk_box_pack_start(GTK_BOX(fg_box), frame, FALSE, TRUE, 0 );
        
        wList = g_slist_append(wList,frame);
        history.push_back(WM_behavior_graph(target,view,frame));
        
        //g_signal_connect(window, "delete-event", G_CALLBACK( delete_event_fg ), (gpointer) gtarget);
        //g_signal_connect (G_OBJECT (view), "destroy", G_CALLBACK (destroy_fg), NULL); //, wList);
        
        g_signal_connect (G_OBJECT (x_button), "clicked", G_CALLBACK (close_fgbox), (gpointer)frame);
        g_signal_connect( G_OBJECT( col_combo ), "changed",G_CALLBACK( col_combo_changed ), (gpointer)frame );
        
        //gtk_widget_show(view);
        gtk_widget_show_all(frame);
        
        //g_timeout_add_seconds(1,(GSourceFunc)plot_fg, (gpointer) gtarget );
        //std::cout<<"DONE\n";
    }
    static gboolean show_update(GtkWidget *image_frame){
        //image show
        //GtkWidget *image = gtk_image_new_from_file("/home/hargalaten/noname.dot.png");
        
        GError *error = NULL;
        GdkPixbuf *pixbuf;
        
        
        //pthread_mutex_lock(&memMutex);
        pixbuf = gdk_pixbuf_new_from_file_at_size ("/home/hargalaten/noname.dot.png", image_frame->allocation.width - 5, image_frame->allocation.height - 5, &error);
        //pixbuf = gdk_pixbuf_new_from_file ("/home/hargalaten/noname.dot.png", &error);
        //pthread_mutex_unlock(&memMutex);
        if (!pixbuf)
        {
          g_print ("Error: %s\n", error->message);
          g_error_free (error);
          /* Handle error here */
          //g_object_unref (pixbuf);
          return TRUE;
        }

        GtkWidget *image = gtk_image_new_from_pixbuf(pixbuf);
        g_object_unref (pixbuf);
        
        //std::cout<<"show-update\n";
        if(oldImage != NULL){
            //std::cout<<"oldImg removed\n";
            gtk_container_remove(GTK_CONTAINER (image_frame), oldImage);
        }
        gtk_container_add (GTK_CONTAINER (image_frame), image);
        oldImage = image;
        gtk_widget_show (image);
        
        return TRUE;
    }
    gboolean delete_event( GtkWidget *widget,
                              GdkEvent  *event,
                              gpointer   data )
    {
        /* If you return FALSE in the "delete-event" signal handler,
         * GTK will emit the "destroy" signal. Returning TRUE means
         * you don't want the window to be destroyed.
         * This is useful for popping up 'are you sure you want to quit?'
         * type dialogs. */

        g_print ("delete event occurred\n");

        /* Change TRUE to FALSE and the main window will be destroyed with
         * a "delete-event". */

        return FALSE;
    }
    static void destroy( GtkWidget *widget,
                         gpointer   data )
    {
        gtk_main_quit ();
    }
    static gboolean resize_image(GtkWidget *widget, GdkEvent *event, GtkWidget *window)
    {
            GdkPixbuf *pixbuf =	gtk_image_get_pixbuf(GTK_IMAGE(widget));
            if (pixbuf == NULL)
            {
                    g_printerr("Failed to resize image\n");
                    return 1;
            }

            //printf("Width: %i\nHeight%i\n", widget->allocation.width, widget->allocation.height);

            pixbuf = gdk_pixbuf_scale_simple(pixbuf, widget->allocation.width, widget->allocation.height, GDK_INTERP_BILINEAR);

            gtk_image_set_from_pixbuf(GTK_IMAGE(widget), pixbuf);
            
            g_object_unref (pixbuf);

            return FALSE;
    }

    void start() {
        char ** args[] = {
        };
        
        int *argsc = new int( sizeof(args)/sizeof(char*) ); 
        
        //gtk3
        GtkWidget *window, *lbox, *button, *entry;
        gtk_init(argsc, args);
        std::cout<<"START\n";
        gtkBuilder = gtk_builder_new();
        wList = g_slice_new (GSList);
        
        gtk_builder_add_from_file(gtkBuilder,"/home/hargalaten/glade_ws/seed_gui_2.glade",NULL);
        window = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "seed_gui"));
        button = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "button_fg"));
        entry = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "behavior_entry"));
        //gtk_widget_set_size_request(entry,100,25);
        
        GtkWidget * show = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "show_frame"));
        fg_total_view = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "fg_total_view"));
        gtk_widget_set_size_request(fg_total_view,410,150);
        GtkWidget * fg_box = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "fg_viewport"));
        //gtk_widget_set_usize(lbox,lbox->allocation.width,25);
        ///gtk_widget_set_size_request(fg_box,410,200);
        
        
        g_timeout_add_seconds(1,(GSourceFunc)show_update, (gpointer)show);
        g_timeout_add_seconds(1,(GSourceFunc)plot_fg, NULL );
        /* attach standard event handlers */
	g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
	//g_signal_connect(show, "expose-event", G_CALLBACK(resize_image), (gpointer)window);
        g_signal_connect (G_OBJECT (button), "clicked", G_CALLBACK (load_fgbox_widget), (gpointer)entry);
        
        lbox = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "lower_box"));
        //gtk_widget_set_usize(lbox,lbox->allocation.width,25);
        gtk_widget_set_size_request(lbox,100,25);
        
        GtkWidget *lb1 = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "button_l1"));
        gtk_widget_set_size_request(lb1,100,25);
        
        GtkWidget *lb2 = GTK_WIDGET(gtk_builder_get_object(gtkBuilder, "button_l2"));
        gtk_widget_set_size_request(lb2,100,25);
        
        gtk_widget_show(window);
    }
    /* This is a callback function to watch data in standard output and write to a view text widget. */
    static void destroy_fg( GtkWidget *widget )
    {
        std::cout<<"DELETE\n";
        std::cout<<"delete EVENT for "<< gtk_window_get_title(GTK_WINDOW(widget) ) <<"\n";
        
        wList = g_slist_remove(wList, widget);
        
        for(int i=0; i<history.size(); i++){
            //if(gtk_window_get_title(GTK_WINDOW(history[i].window)) == gtk_window_get_title(GTK_WINDOW(widget)) ){
            if(history[i].window == widget ){
                std::cout<<"deleteing: "<<history[i].schema<<"\n";
                g_object_unref(G_OBJECT(history[i].builder));
                history.erase(history.begin() + i);
                i--;
            }
        }
        
    }
    static void col_combo_changed( GtkWidget *widget, GtkWidget *fgbox ){
        for(int i=0; i<history.size(); i++){
            if(history[i].window == fgbox ){
                GdkColor *newColor;
                std::string selectedCol = gtk_combo_box_get_active_text(GTK_COMBO_BOX(widget));
                history[i].color = colorMap[selectedCol];
            }
        }
    }
    static void close_fgbox( GtkWidget *widget, GtkWidget *fgbox )
    {
        std::cout<<"DELETE\n";
        //std::cout<<"delete EVENT for "<< gtk_window_get_title(GTK_WINDOW(widget) ) <<"\n";
        
        wList = g_slist_remove(wList, fgbox);
        
        for(int i=0; i<history.size(); i++){
            //if(gtk_window_get_title(GTK_WINDOW(history[i].window)) == gtk_window_get_title(GTK_WINDOW(widget)) ){
            if(history[i].window == fgbox ){
                std::cout<<"deleteing: "<<history[i].schema<<"\n";
                gtk_widget_destroy(fgbox);
                g_object_unref(G_OBJECT(history[i].builder));
                history.erase(history.begin() + i);
                i--;
            }
        }
        
    }
    static void stdout_callback( gpointer          data,
                        gint              source,
                        GdkInputCondition condition )
    {
       gchar buf[1024];
       gint chars_read;
       GtkTextIter iter;
       chars_read = 1024;
       gtk_text_buffer_get_end_iter(buffer, &iter);
       while (chars_read == 1024){
         chars_read = read(fds[0], buf, 1024);
         
         // fprintf(stderr, "%i chars: %s\n", chars_read, buf);
         gtk_text_buffer_insert (buffer, &iter, buf, chars_read);
       }
    }
    void exit(){
         g_object_unref(G_OBJECT(gtkBuilder));
         g_slist_free (wList);
    }
    static WM_node * getReleasedInstance( std::string targetInstance ){
        int j = 0;
        //se esiste, prendi l'istanza rilasciata
        while (j < WM->getNodesByInstance(targetInstance).size() &&
                !WM->getNodesByInstance(targetInstance)[j]->isBranchReleased())
            j++;

        WM_node *pivot;

        if (j == 0){
            pivot = WM->getNodesByInstance(targetInstance)[j];
        }
        else{
            pivot = WM->getNodesByInstance(targetInstance)[j - 1];
        }
        
        return pivot;
    }
protected:
    static GtkWidget *oldImage;
    static GtkBuilder * gtkBuilder;
    /* The Text Buffer as a global variable*/
    static GtkTextBuffer *buffer;
    /* File descriptors for pipe. */
    static int fds[2];
    
    static std::vector< WM_behavior_graph >  history;
    //static pthread_mutex_t gui_mutex;
    static GSList *wList;
    static GtkWidget *oldFg, *fg_total_view;
    static std::map<std::string, cv::Scalar> colorMap;
};

GtkWidget * guiBehavior::oldImage = NULL;
GtkTextBuffer * guiBehavior::buffer = NULL;
int guiBehavior::fds[2] = {0,0};
std::vector< WM_behavior_graph > guiBehavior::history = std::vector< WM_behavior_graph >();
std::map<std::string, cv::Scalar> guiBehavior::colorMap = std::map<std::string, cv::Scalar>();
GSList * guiBehavior::wList = NULL; 
GtkBuilder * guiBehavior::gtkBuilder = NULL;
GtkWidget * guiBehavior::oldFg = NULL;
GtkWidget * guiBehavior::fg_total_view = NULL;
