#!/usr/bin/env python3
import json
import math
import os
import sys
import time
from PyQt5 import QtWidgets, QtGui, QtCore

seed_name = None

def get_ros():
    """Importa rclpy e String solo quando necessario."""
    import rclpy
    from std_msgs.msg import String
    return rclpy, String


#   GRAPH VIEW

class GraphView(QtWidgets.QGraphicsView):
    nodeClicked = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setScene(QtWidgets.QGraphicsScene())
        self.setRenderHint(QtGui.QPainter.Antialiasing)

        self.nodes = {}

        self.min_node_width = 110
        self.node_height = 40
        self.x_spacing = 220  # Spaziatura orizzontale base tra nodi
        self.y_spacing = 120  # Spaziatura verticale base tra livelli
        
        # Parametri aggiuntivi per migliorare la spaziatura
        self.min_x_spacing = 180  # Spaziatura minima tra nodi
        self.max_x_spacing = 350  # Spaziatura massima tra nodi
        self.base_tree_spacing = 250  # Spaziatura base tra alberi diversi
        self.auto_spacing_enabled = True  # Abilita spaziatura automatica

        # Zoom limits
        self.min_scale = 0.1
        self.max_scale = 5.0

        # Panning
        self._panning = False
        self._pan_start = QtCore.QPoint()
        self.setCursor(QtCore.Qt.ArrowCursor)

        # Attiva scroll verticale/orizzontale
        self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        
        # Auto-fit
        self._auto_fit = False

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._panning = True
            self._pan_start = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            event.accept()
            return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()

            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - delta.x()
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - delta.y()
            )

            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self._panning:
            self._panning = False
            self.setCursor(QtCore.Qt.ArrowCursor)
            event.accept()
            return

        super().mouseReleaseEvent(event)

    def wheelEvent(self, event):
        if event.modifiers() == QtCore.Qt.ControlModifier:
            angle = event.angleDelta().y()
            factor = 1.15 if angle > 0 else 1 / 1.15

            current_scale = self.transform().m11()
            new_scale = current_scale * factor

            if self.min_scale < new_scale < self.max_scale:
                self.scale(factor, factor)

            return

        super().wheelEvent(event)

    def clear_graph(self):
        self.scene().clear()
        self.nodes = {}

    def draw_graph(self, wm_map):
        self.clear_graph()
        if not wm_map:
            return

        children = {name: [] for name in wm_map}
        roots = []

        for name, data in wm_map.items():
            father = data.get("father", "none")
            if father == "none" or father not in wm_map:
                roots.append(name)
            else:
                children[father].append(name)

        # Calcola la larghezza totale necessaria per tutti gli alberi
        total_width = 0
        tree_widths = []
        
        for root in roots:
            tree_width = self._calculate_tree_width(root, children, wm_map)
            tree_widths.append(tree_width)
            total_width += tree_width + self.base_tree_spacing
        
        # Disegna ogni albero con spaziatura adeguata
        current_x = 0
        for i, root in enumerate(roots):
            tree_x = current_x + tree_widths[i] / 2
            current_x += tree_widths[i] + self.base_tree_spacing
            self._draw_tree(root, children, wm_map, tree_x, 0, tree_widths[i], True, False)

        self.scene().setSceneRect(self.scene().itemsBoundingRect())

        if self._auto_fit:
            self.fitInView(self.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def _calculate_tree_width(self, node, children, wm_map, level=0):
        """Calcola la larghezza totale richiesta da un sottoalbero"""
        num_children = len(children[node])
        if num_children == 0:
            return self.min_node_width
        
        # Per nodi con figli, calcola la larghezza come somma delle larghezze dei figli
        child_widths = []
        for child in children[node]:
            child_widths.append(self._calculate_tree_width(child, children, wm_map, level + 1))
        
        total_child_width = sum(child_widths)
        
        # Aggiungi spaziatura tra i figli
        if num_children > 1:
            total_child_width += (num_children - 1) * self._get_adjusted_spacing(num_children, level)
        
        return max(self.min_node_width, total_child_width)

    def _get_adjusted_spacing(self, num_children, level):
        """Calcola la spaziatura in base al numero di figli e al livello"""
        if not self.auto_spacing_enabled:
            return self.x_spacing
        
        # Aumenta la spaziatura per nodi con molti figli
        base_spacing = self.x_spacing
        if num_children > 3:
            # Spaziatura progressiva per molti figli
            spacing_factor = min(1.5, 1.0 + (num_children - 3) * 0.1)
            base_spacing *= spacing_factor
        
        return min(self.max_x_spacing, max(self.min_x_spacing, base_spacing))

    def _format_formula_list(self, formula_list):
        if not formula_list:
            return ""
        converted = []
        for s in formula_list:
            s = str(s)
            if s.startswith("-"):
                s = "¬" + s[1:]
            converted.append(s)
        return " Ʌ ".join(converted)

    def _draw_tree(self, name, children, wm_map, x, y, tree_width, father_releaser, father_goal):
        wm = wm_map[name]

        releaser_flag = bool(wm.get("releaser", False))
        goal_flag = bool(wm.get("goal", False))
        emphasis = float(wm.get("emphasis", 0.0))
        abstract = bool(wm.get("abstract", False))

        if goal_flag or father_goal:
            border_color = QtGui.QColor("blue")
        elif not releaser_flag or not father_releaser:
            border_color = QtGui.QColor("red")
        else:
            border_color = QtGui.QColor("green")

        fill_color = QtGui.QColor("lightGray" if abstract else "darkGray")

        font = self.font()
        fm = QtGui.QFontMetrics(font)

        name_text = name
        emp_text = f"({emphasis:.2f})"

        name_width = fm.horizontalAdvance(name_text)
        emp_width = fm.horizontalAdvance(emp_text)
        max_text = max(name_width, emp_width)

        node_width = max(self.min_node_width, max_text + 20)

        rect_item = QtWidgets.QGraphicsEllipseItem(
            -node_width / 2, -self.node_height / 2, node_width, self.node_height
        )
        rect_item.setBrush(QtGui.QBrush(fill_color))
        rect_item.setPen(QtGui.QPen(border_color, 3))
        rect_item.setPos(x, y)
        self.scene().addItem(rect_item)

        dy = fm.height() / 4 + 2

        name_item = QtWidgets.QGraphicsTextItem(name_text)
        name_item.setFont(font)
        name_item.setDefaultTextColor(QtCore.Qt.black)
        brn = name_item.boundingRect()
        name_item.setPos(x - brn.width() / 2, y - brn.height() / 2 - dy)
        self.scene().addItem(name_item)

        emp_item = QtWidgets.QGraphicsTextItem(emp_text)
        emp_item.setFont(font)
        emp_item.setDefaultTextColor(QtCore.Qt.black)
        bre = emp_item.boundingRect()
        emp_item.setPos(x - bre.width() / 2, y - bre.height() / 2 + dy)
        self.scene().addItem(emp_item)

        # CLICK
        rect_item.setData(0, name)
        rect_item.mousePressEvent = lambda event: self.nodeClicked.emit(name)

        self.nodes[name] = (rect_item, name_item, x, y, node_width)

        # CHILDREN
        num_children = len(children[name])
        if num_children != 0:

            # Calcola spaziatura adattiva in base al numero di figli
            spacing = self._get_adjusted_spacing(num_children, 0)
            
            # Calcola la larghezza totale occupata dai figli
            total_children_width = 0
            child_widths = []
            for child in children[name]:
                child_tree_width = self._calculate_tree_width(child, children, wm_map, 1)
                child_widths.append(child_tree_width)
                total_children_width += child_tree_width
            
            # Aggiungi spaziatura tra i figli
            if num_children > 1:
                total_children_width += (num_children - 1) * spacing
            
            # Posiziona i figli in modo centrato sotto il padre
            current_x = x - total_children_width / 2
            for i, son in enumerate(children[name]):
                child_tree_width = child_widths[i]
                cx = current_x + child_tree_width / 2
                cy = y + self.y_spacing
                
                self._draw_tree(son, children, wm_map, cx, cy, child_tree_width, releaser_flag and father_releaser, goal_flag or father_goal)
                
                # Disegna il bordo
                self._draw_edge(x, y + self.node_height / 2,
                                cx, cy - self.node_height / 2,
                                goal_flag or father_goal, releaser_flag and father_releaser)
                
                current_x += child_tree_width + spacing
        
        # BOXES

        # Releaser box
        rel_formulae = wm.get("releaser_formulae", [])
        rel_text = self._format_formula_list(rel_formulae) or "TRUE"

        rel_font = QtGui.QFont(font)
        rel_font.setPointSize(max(font.pointSize() - 2, 6))

        rel_item = QtWidgets.QGraphicsTextItem(rel_text)
        rel_item.setFont(rel_font)
        rel_item.setDefaultTextColor(QtCore.Qt.black)
        br_rel = rel_item.boundingRect()

        bg_w = br_rel.width() + 8
        bg_h = br_rel.height() + 4
        bg_x = x - bg_w / 2
        bg_y = y - self.node_height / 2 - bg_h - 5

        rel_bg = QtWidgets.QGraphicsRectItem(0, 0, bg_w, bg_h)
        rel_bg.setBrush(QtGui.QBrush(QtGui.QColor("green") if releaser_flag else QtGui.QColor("red")))
        rel_bg.setPen(QtGui.QPen(QtCore.Qt.black, 1))
        rel_bg.setPos(bg_x, bg_y)
        self.scene().addItem(rel_bg)

        rel_item.setParentItem(rel_bg)
        rel_item.setPos((bg_w - br_rel.width()) / 2, (bg_h - br_rel.height()) / 2)

        # Goal box
        goal_formulae = wm.get("goal_formulae", [])
        goal_text = self._format_formula_list(goal_formulae) or "NONE"

        goal_font = QtGui.QFont(font)
        goal_font.setPointSize(max(font.pointSize() - 2, 6))

        goal_item = QtWidgets.QGraphicsTextItem(goal_text)
        goal_item.setFont(goal_font)
        goal_item.setDefaultTextColor(QtCore.Qt.black)
        br_rel = goal_item.boundingRect()

        bg_w = br_rel.width() + 8
        bg_h = br_rel.height() + 4
        bg_x = x - bg_w / 2
        bg_y = y + self.node_height / 2 + 5 # bg_h + 5

        goal_bg = QtWidgets.QGraphicsRectItem(0, 0, bg_w, bg_h)
        goal_bg.setBrush(QtGui.QBrush(QtGui.QColor("blue") if goal_flag else QtGui.QColor("gray")))
        goal_bg.setPen(QtGui.QPen(QtCore.Qt.black, 1))
        goal_bg.setPos(bg_x, bg_y)
        self.scene().addItem(goal_bg)

        goal_item.setParentItem(goal_bg)
        goal_item.setPos((bg_w - br_rel.width()) / 2, (bg_h - br_rel.height()) / 2)

    def _draw_edge(self, x1, y1, x2, y2, goal_flag, releaser_flag):
        line = QtCore.QLineF(x1, y1, x2, y2)

        if goal_flag:
            color = QtGui.QColor("blue")
        elif not releaser_flag:
            color = QtGui.QColor("red")
        else:
            color = QtGui.QColor("green")

        pen = QtGui.QPen(color, 2)
        item = QtWidgets.QGraphicsLineItem(line)
        item.setPen(pen)
        self.scene().addItem(item)

        dx, dy = line.dx(), line.dy()
        angle = math.atan2(-dy, dx)
        arrow = 10

        p1 = line.p2() + QtCore.QPointF(math.sin(angle - math.pi / 3) * arrow,
                                        math.cos(angle - math.pi / 3) * arrow)
        p2 = line.p2() + QtCore.QPointF(math.sin(angle - math.pi + math.pi / 3) * arrow,
                                        math.cos(angle - math.pi + math.pi / 3) * arrow)

        poly = QtGui.QPolygonF([line.p2(), p1, p2])
        arr = QtWidgets.QGraphicsPolygonItem(poly)
        arr.setBrush(QtGui.QBrush(color))
        arr.setPen(QtGui.QPen(color))
        self.scene().addItem(arr)

    # ZOOM

    def wheelEvent(self, event):
        self._auto_fit = False
        delta = event.angleDelta().y()
        if delta == 0:
            return

        factor = 1.15 if delta > 0 else 1 / 1.15
        current = self.transform().m11()
        new = current * factor
        if 0.1 < new < 5.0:
            self.scale(factor, factor)



class SeedTree(QtWidgets.QWidget):
    # Signal per ricevere dati da ROS in modo thread-safe
    wm_data_received = QtCore.pyqtSignal(str)
    
    def __init__(self, json_path="/tmp/seed_wm.json", refresh_interval=2000):
        super().__init__()

        self.json_path = json_path
        self.details_visible = False
        self.ros_node = None
        self.pub = None
        self.sub = None
        self.use_ros = True  # Flag per usare ROS o fallback a file

        self.setWindowTitle("SEED Working Memory Viewer")
        self.resize(1600, 900)

        main_layout = QtWidgets.QVBoxLayout(self)

        # TOP BAR
        top = QtWidgets.QHBoxLayout()
        self.label_status = QtWidgets.QLabel("Initializing ROS connection...")
        top.addWidget(self.label_status)
        top.addStretch()
        main_layout.addLayout(top)

        # SPLITTER (Tree | Graph | Details)
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        main_layout.addWidget(self.splitter, 1)

        # TREE
        self.tree = QtWidgets.QTreeView()
        self.model = QtGui.QStandardItemModel()
        self.model.setHorizontalHeaderLabels(["wm", "Emphasis", "Releaser", "Goal", "Truth"])
        self.tree.setModel(self.model)
        self.tree.selectionModel().selectionChanged.connect(self.on_selection_changed)
        self.splitter.addWidget(self.tree)
        
        # IMPOSTAZIONI DELL'ALBERO PER VISUALIZZARE I NOMI COMPLETI
        self.tree.setHeaderHidden(False)
        self.tree.setAlternatingRowColors(True)
        self.tree.setAnimated(True)
        self.tree.setAllColumnsShowFocus(True)
        
        # Imposta larghezza minima per le colonne
        self.tree.setColumnWidth(0, 250)  # wm - colonna più larga
        self.tree.setColumnWidth(1, 80)   # Emphasis
        self.tree.setColumnWidth(2, 80)   # Releaser
        self.tree.setColumnWidth(3, 60)   # Goal
        self.tree.setColumnWidth(4, 60)   # Truth
        
        # Permetti il ridimensionamento delle colonne
        self.tree.header().setStretchLastSection(False)
        self.tree.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Interactive)  # wm resizable
        self.tree.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)  # Emphasis auto-size
        self.tree.header().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)  # Releaser auto-size
        self.tree.header().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)  # Goal auto-size
        self.tree.header().setSectionResizeMode(4, QtWidgets.QHeaderView.ResizeToContents)  # Truth auto-size
        
        # Abilita word wrap per i nomi lunghi
        self.tree.setWordWrap(True)

        # GRAPH
        self.graph = GraphView()
        self.graph.nodeClicked.connect(self.on_graph_node_clicked)
        self.splitter.addWidget(self.graph)

        # DETAILS PANEL
        self.details_widget = QtWidgets.QWidget()
        details_layout = QtWidgets.QVBoxLayout(self.details_widget)
        
        # Title bar with close button
        title_bar = QtWidgets.QHBoxLayout()
        self.details_title = QtWidgets.QLabel("<b>Seleziona un wm</b>")
        title_bar.addWidget(self.details_title)
        title_bar.addStretch()
        
        # Close button
        self.btn_close_details = QtWidgets.QPushButton("✕")
        self.btn_close_details.setFixedSize(25, 25)
        self.btn_close_details.setToolTip("Chiudi dettagli")
        self.btn_close_details.clicked.connect(self.hide_details)
        title_bar.addWidget(self.btn_close_details)
        
        details_layout.addLayout(title_bar)
        
        # Details text
        self.details_text = QtWidgets.QTextEdit()
        self.details_text.setReadOnly(True)
        details_layout.addWidget(self.details_text)
        
        self.splitter.addWidget(self.details_widget)
        self.details_widget.hide()

        self.splitter.setSizes([500, 800, 0])

        # COMMAND BAR
        cmd_layout = QtWidgets.QHBoxLayout()

        self.cmd_input = QtWidgets.QLineEdit()
        self.cmd_input.setPlaceholderText("Enter SEED command")
        cmd_layout.addWidget(self.cmd_input)

        self.btn_send_cmd = QtWidgets.QPushButton("SEND")
        cmd_layout.addWidget(self.btn_send_cmd)

        main_layout.addLayout(cmd_layout)

        self.btn_send_cmd.clicked.connect(self.send_raw_cmd)
        
        # Connette il signal per ricevere dati ROS
        self.wm_data_received.connect(self.update_from_ros)

        # Inizializza ROS
        self.init_ros()
        
        # Timer per processare callback ROS
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(100)  # 10Hz per processare callback ROS

    #  ROS2

    def init_ros(self):
        """Inizializza ROS con publisher per comandi e subscriber per dati"""
        try:
            rclpy, String = get_ros()
            rclpy.init(args=None)
            self.ros_node = rclpy.create_node("seed_" + seed_name + "_gui_py")
            
            # PUBLISHER per inviare comandi a SEED 
            self.pub = self.ros_node.create_publisher(String, "/seed_" + seed_name + "/stream", 10)
            
            # SUBSCRIBER per ricevere dati dal nodo C++
            self.sub = self.ros_node.create_subscription(
                String,
                "/seed_" + seed_name + "/wm", 
                self.on_wm_received,
                10
            )
            
            self.label_status.setText("Connected to ROS - Waiting for SEED data...")
            self.use_ros = True
            
        except Exception as e:
            self.label_status.setText(f"ROS Error: {e} - Falling back to file reading")
            self.use_ros = False

    def spin_ros(self):
        """Processa callback ROS periodicamente"""
        if self.ros_node:
            rclpy, _ = get_ros()
            rclpy.spin_once(self.ros_node, timeout_sec=0.01)

    def on_wm_received(self, msg):
        """Callback quando arrivano dati dal publisher C++"""
        # Invia i dati al thread principale Qt tramite signal
        self.wm_data_received.emit(msg.data)

    #  ROS2 SEND COMMAND

    def send_raw_cmd(self):
        text = self.cmd_input.text().strip()
        if not text:
            return

        try:
            if self.ros_node is None:
                self.init_ros()
                if self.ros_node is None:
                    raise Exception("ROS not available")
            
            _, String = get_ros()
            msg = String()
            msg.data = text
            self.pub.publish(msg)

            self.label_status.setText(f"Command sent: {text}")
            self.cmd_input.clear()

        except Exception as e:
            self.label_status.setText(f"ERROR sending command: {e}")

    #  UPDATE GUI

    def update_from_ros(self, json_str):
        """Aggiorna la GUI con i dati ricevuti da ROS"""
        try:
            data = json.loads(json_str)
            self.update_gui_with_data(data)
            
        except json.JSONDecodeError as e:
            self.label_status.setText(f"Invalid JSON from ROS: {e}")
        except Exception as e:
            self.label_status.setText(f"Error processing ROS data: {e}")

    def update_gui_with_data(self, data):
        """Aggiorna tree e graph con i dati ricevuti"""
        wm = data.get("wm", [])
        if not wm:
            self.label_status.setText("No wm in data")
            return
            
        self.wm_map = {b["name"]: b for b in wm}

       
        expanded = set()
        def save_expand(idx=QtCore.QModelIndex()):
            for i in range(self.model.rowCount(idx)):
                c = self.model.index(i, 0, idx)
                if self.tree.isExpanded(c):
                    expanded.add(self.model.data(c))
                save_expand(c)
        save_expand()

        # Pulisci e ricrea il tree
        self.model.removeRows(0, self.model.rowCount())
        item_map = {}

        # Popola il tree
        for b in wm:
            name = b["name"]
            emp = f"{b.get('emphasis', 0):.2f}"
            rel = str(b.get("releaser", False))
            goal = str(b.get("goal", False))
            truth = str(b.get("truth", True))

            name_item = QtGui.QStandardItem(name)
            emp_item = QtGui.QStandardItem(emp)
            rel_item = QtGui.QStandardItem(rel)
            goal_item = QtGui.QStandardItem(goal)
            truth_item = QtGui.QStandardItem(truth)

            # Colori in base allo stato
            if goal == "True":
                name_item.setForeground(QtGui.QColor("blue"))
            elif rel == "True":
                name_item.setForeground(QtGui.QColor("green"))
            elif truth == "False":
                name_item.setForeground(QtGui.QColor("red"))
            else:
                name_item.setForeground(QtGui.QColor("gray"))

            item_map[name] = (name_item, emp_item, rel_item, goal_item, truth_item)

        # Costruisce la gerarchia
        roots = []
        for name, row in item_map.items():
            father = self.wm_map[name].get("father", "none")
            if father == "none" or father not in item_map:
                roots.append(row)
            else:
                item_map[father][0].appendRow(list(row))

        for r in roots:
            self.model.appendRow(list(r))

        
        def restore(idx=QtCore.QModelIndex()):
            for i in range(self.model.rowCount(idx)):
                c = self.model.index(i, 0, idx)
                if self.model.data(c) in expanded:
                    self.tree.setExpanded(c, True)
                restore(c)
        restore()

        # AGGIORNA LA LARGHEZZA DELLE COLONNE DOPO AVER POPOLATO L'ALBERO
        # Calcola la larghezza massima per i nomi
        font_metrics = self.tree.fontMetrics()
        max_width = 0
        for b in wm:
            width = font_metrics.horizontalAdvance(b["name"]) + 50  # 50px di margine
            max_width = max(max_width, width)
        
        # Imposta una larghezza minima per la colonna "wm"
        self.tree.setColumnWidth(0, max(250, min(max_width, 400)))  # Min 250px, max 400px

        # Aggiorna il grafico
        self.graph.draw_graph(self.wm_map)

        self.label_status.setText(f"ROS: {len(wm)} wm — {time.strftime('%H:%M:%S')}")



    

    def on_selection_changed(self, selected, deselected):
        if selected.indexes():
            self.show_details(self.model.data(selected.indexes()[0]))

    def on_graph_node_clicked(self, name):
        self.show_details(name)
        self.reselect_wm(name)

    def reselect_wm(self, name):
        def search(idx):
            for i in range(self.model.rowCount(idx)):
                c = self.model.index(i, 0, idx)
                if self.model.data(c) == name:
                    self.tree.setCurrentIndex(c)
                    return True
                if search(c):
                    return True
            return False
        search(QtCore.QModelIndex())

    def show_details(self, name):
        if not hasattr(self, 'wm_map') or name not in self.wm_map:
            return

        if not self.details_visible:
            self.details_widget.show()
            self.splitter.setSizes([600, 600, 400])
            self.details_visible = True

        wm = self.wm_map[name]
        self.details_title.setText(f"<b>wm:</b> {name}")

        lines = []
        for k, v in wm.items():
            lines.append(f"<b>{k}</b>: {json.dumps(v, indent=2, ensure_ascii=False)}")

        self.details_text.setHtml("<br>".join(lines))

    def hide_details(self):
        """Chiude il pannello dei dettagli"""
        self.details_widget.hide()
        self.splitter.setSizes([600, 1000, 0])
        self.details_visible = False

    def closeEvent(self, event):
        """Pulizia alla chiusura"""
        if self.ros_node:
            try:
                rclpy, _ = get_ros()
                rclpy.shutdown()
            except:
                pass
        event.accept()


# MAIN

def main():
    app = QtWidgets.QApplication(sys.argv)

    global seed_name
    if len(sys.argv) > 1:
        seed_name = sys.argv[1]
        print(f"SEED name is: {seed_name}")
    
    gui = SeedTree()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
