#include "mainwindow.h"
#include "MapView.h"

#include <QStackedWidget>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QLabel>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QFrame>

#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsSimpleTextItem>
#include <QPen>
#include <QBrush>

static const char* kUserFile = "users.user";

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    buildUi();
    applyTheme();

    userManager.load(kUserFile);
    userManager.ensureDefaultAdmin(kUserFile);

    resize(1120, 720);
    showLoginPage();
}

long long MainWindow::edgeKey(int u, int v) const {
    if (u > v) std::swap(u, v);
    return static_cast<long long>(u) * 1000000LL + v;
}

void MainWindow::applyTheme() {
    setStyleSheet(R"(
QMainWindow { background: #FFF6D6; }

QLabel { color: #3F2F1F; font-size: 14px; }
QLabel#titleLabel { font-size: 22px; font-weight: 700; }
QLabel#subtitleLabel { font-size: 16px; font-weight: 700; }
QLabel#hintLabel { color: #6B563A; font-size: 13px; }

QWidget#card {
    background: #FFFEF6;
    border: 1px solid #E8D7A8;
    border-radius: 18px;
}

QLineEdit {
    background: #FFFFFF;
    border: 1px solid #E3D0A0;
    border-radius: 12px;
    padding: 10px 12px;
    font-size: 14px;
    selection-background-color: #FFE08A;
}

QTextEdit {
    background: #FFFFFF;
    border: 1px solid #E3D0A0;
    border-radius: 14px;
    padding: 12px;
    font-size: 14px;
}

QPushButton {
    background: #FFE08A;
    border: 1px solid #D9B86B;
    border-radius: 14px;
    padding: 10px 18px;
    font-weight: 700;
}
QPushButton:hover { background: #FFD56A; }
QPushButton:pressed { background: #F3C757; }

QPushButton#secondaryBtn {
    background: #FFF2C2;
    border: 1px solid #D9C48A;
}
QPushButton#secondaryBtn:hover { background: #FFE8A8; }

QRadioButton { color: #3F2F1F; font-size: 14px; }

QFrame#divider { background: #E8D7A8; }
)");
}

void MainWindow::buildUi() {
    stackedWidget = new QStackedWidget(this);
    setCentralWidget(stackedWidget);

    // ================== Login Page ==================
    QWidget* loginPage = new QWidget();
    QVBoxLayout* pageRoot = new QVBoxLayout(loginPage);
    pageRoot->setContentsMargins(28, 28, 28, 28);

    QWidget* loginCard = new QWidget();
    loginCard->setObjectName("card");
    QVBoxLayout* cardLayout = new QVBoxLayout(loginCard);
    cardLayout->setContentsMargins(28, 24, 28, 24);
    cardLayout->setSpacing(14);

    QLabel* title = new QLabel("地图导航系统");
    title->setObjectName("titleLabel");
    cardLayout->addWidget(title);

    QLabel* subtitle = new QLabel("登录");
    subtitle->setObjectName("subtitleLabel");
    cardLayout->addWidget(subtitle);

    QFormLayout* form = new QFormLayout();
    form->setHorizontalSpacing(14);
    form->setVerticalSpacing(12);

    loginUserEdit = new QLineEdit();
    loginPassEdit = new QLineEdit();
    loginPassEdit->setEchoMode(QLineEdit::Password);
    loginUserEdit->setPlaceholderText("输入用户名");
    loginPassEdit->setPlaceholderText("输入密码");

    form->addRow("用户名：", loginUserEdit);
    form->addRow("密码：", loginPassEdit);
    cardLayout->addLayout(form);

    QHBoxLayout* roleRow = new QHBoxLayout();
    adminRadio = new QRadioButton("管理员");
    userRadio = new QRadioButton("普通用户");
    userRadio->setChecked(true);
    roleRow->addWidget(adminRadio);
    roleRow->addWidget(userRadio);
    roleRow->addStretch();
    cardLayout->addLayout(roleRow);

    QLabel* hint = new QLabel("管理员账号由系统初始化创建，请勿注册管理员。");
    hint->setObjectName("hintLabel");
    cardLayout->addWidget(hint);

    QHBoxLayout* btnRow = new QHBoxLayout();
    loginButton = new QPushButton("登录");
    goRegisterButton = new QPushButton("去注册");
    goRegisterButton->setObjectName("secondaryBtn");
    btnRow->addWidget(loginButton);
    btnRow->addWidget(goRegisterButton);
    btnRow->addStretch();
    cardLayout->addLayout(btnRow);

    QHBoxLayout* centerRow = new QHBoxLayout();
    centerRow->addStretch();
    centerRow->addWidget(loginCard);
    centerRow->addStretch();
    pageRoot->addStretch();
    pageRoot->addLayout(centerRow);
    pageRoot->addStretch();

    loginCard->setFixedWidth(520);

    // ================== Register Page ==================
    QWidget* registerPage = new QWidget();
    QVBoxLayout* regPageRoot = new QVBoxLayout(registerPage);
    regPageRoot->setContentsMargins(28, 28, 28, 28);

    QWidget* regCard = new QWidget();
    regCard->setObjectName("card");
    QVBoxLayout* regLayout = new QVBoxLayout(regCard);
    regLayout->setContentsMargins(28, 24, 28, 24);
    regLayout->setSpacing(14);

    QLabel* regTitle = new QLabel("注册普通用户");
    regTitle->setObjectName("subtitleLabel");
    regLayout->addWidget(regTitle);

    QFormLayout* regForm = new QFormLayout();
    regForm->setHorizontalSpacing(14);
    regForm->setVerticalSpacing(12);

    regUserEdit = new QLineEdit();
    regPassEdit = new QLineEdit();
    regPass2Edit = new QLineEdit();
    regPassEdit->setEchoMode(QLineEdit::Password);
    regPass2Edit->setEchoMode(QLineEdit::Password);

    regUserEdit->setPlaceholderText("用户名（不能是 admin）");
    regPassEdit->setPlaceholderText("密码");
    regPass2Edit->setPlaceholderText("再次输入密码");

    regForm->addRow("用户名：", regUserEdit);
    regForm->addRow("密码：", regPassEdit);
    regForm->addRow("确认密码：", regPass2Edit);
    regLayout->addLayout(regForm);

    QLabel* regHint = new QLabel("注册仅创建普通用户。管理员请联系管理方获取密码。");
    regHint->setObjectName("hintLabel");
    regLayout->addWidget(regHint);

    QHBoxLayout* regBtnRow = new QHBoxLayout();
    doRegisterButton = new QPushButton("注册");
    backToLoginButton = new QPushButton("返回登录");
    backToLoginButton->setObjectName("secondaryBtn");
    regBtnRow->addWidget(doRegisterButton);
    regBtnRow->addWidget(backToLoginButton);
    regBtnRow->addStretch();
    regLayout->addLayout(regBtnRow);

    regCard->setFixedWidth(520);

    QHBoxLayout* regCenter = new QHBoxLayout();
    regCenter->addStretch();
    regCenter->addWidget(regCard);
    regCenter->addStretch();

    regPageRoot->addStretch();
    regPageRoot->addLayout(regCenter);
    regPageRoot->addStretch();

    // ================== Main Page ==================
    QWidget* mainPage = new QWidget();
    QHBoxLayout* mainRoot = new QHBoxLayout(mainPage);
    mainRoot->setContentsMargins(18, 18, 18, 18);
    mainRoot->setSpacing(14);

    QWidget* mapCard = new QWidget();
    mapCard->setObjectName("card");
    QVBoxLayout* mapLayout = new QVBoxLayout(mapCard);
    mapLayout->setContentsMargins(16, 16, 16, 16);
    mapLayout->setSpacing(10);

    QLabel* mapTitle = new QLabel("地图视图（拖拽移动 / 滚轮缩放 / 点击节点选点）");
    mapTitle->setObjectName("subtitleLabel");
    mapLayout->addWidget(mapTitle);

    mapInfoLabel = new QLabel("未加载地图");
    mapInfoLabel->setObjectName("hintLabel");
    mapLayout->addWidget(mapInfoLabel);

    scene = new QGraphicsScene(this);
    mapView = new MapView();
    mapView->setScene(scene);
    mapView->setMinimumSize(640, 520);
    mapLayout->addWidget(mapView, 1);

    mainRoot->addWidget(mapCard, 2);

    QFrame* divider = new QFrame();
    divider->setObjectName("divider");
    divider->setFixedWidth(2);
    mainRoot->addWidget(divider);

    QWidget* panelCard = new QWidget();
    panelCard->setObjectName("card");
    QVBoxLayout* panelLayout = new QVBoxLayout(panelCard);
    panelLayout->setContentsMargins(16, 16, 16, 16);
    panelLayout->setSpacing(10);

    QLabel* panelTitle = new QLabel("操作面板");
    panelTitle->setObjectName("subtitleLabel");
    panelLayout->addWidget(panelTitle);

    QHBoxLayout* adminRow = new QHBoxLayout();
    adminRow->setSpacing(8);

    loadMapButton = new QPushButton("导入地图");
    saveMapButton = new QPushButton("保存地图");
    addVertexButton = new QPushButton("新增地区");
    removeVertexButton = new QPushButton("删除地区");
    addEdgeButton = new QPushButton("新增道路");
    removeEdgeButton = new QPushButton("删除道路");
    editEdgeButton = new QPushButton("改长度");

    adminRow->addWidget(loadMapButton);
    adminRow->addWidget(saveMapButton);
    adminRow->addWidget(addVertexButton);
    adminRow->addWidget(removeVertexButton);
    adminRow->addWidget(addEdgeButton);
    adminRow->addWidget(removeEdgeButton);
    adminRow->addWidget(editEdgeButton);
    panelLayout->addLayout(adminRow);

    QLabel* clickTip = new QLabel("点击节点自动填入：先起点后终点（再点回到起点）。");
    clickTip->setObjectName("hintLabel");
    panelLayout->addWidget(clickTip);

    QFormLayout* queryForm = new QFormLayout();
    queryForm->setHorizontalSpacing(12);
    queryForm->setVerticalSpacing(10);

    sourceEdit = new QLineEdit();
    targetEdit = new QLineEdit();
    sourceEdit->setPlaceholderText("例如：0（或点击地图节点）");
    targetEdit->setPlaceholderText("例如：3（或点击地图节点）");
    queryForm->addRow("起点ID：", sourceEdit);
    queryForm->addRow("终点ID：", targetEdit);
    panelLayout->addLayout(queryForm);

    QHBoxLayout* runRow = new QHBoxLayout();
    runRow->setSpacing(8);
    runButton = new QPushButton("导航（最短+次短）");
    logoutButton = new QPushButton("退出登录");
    logoutButton->setObjectName("secondaryBtn");
    runRow->addWidget(runButton);
    runRow->addWidget(logoutButton);
    runRow->addStretch();
    panelLayout->addLayout(runRow);

    outputEdit = new QTextEdit();
    outputEdit->setReadOnly(true);
    outputEdit->setMinimumHeight(260);
    panelLayout->addWidget(outputEdit, 1);

    mainRoot->addWidget(panelCard, 1);

    // pages
    stackedWidget->addWidget(loginPage);     // 0
    stackedWidget->addWidget(registerPage);  // 1
    stackedWidget->addWidget(mainPage);      // 2

    // connect
    connect(loginButton, &QPushButton::clicked, this, &MainWindow::onLoginClicked);
    connect(goRegisterButton, &QPushButton::clicked, this, &MainWindow::onGoRegisterClicked);
    connect(doRegisterButton, &QPushButton::clicked, this, &MainWindow::onDoRegisterClicked);
    connect(backToLoginButton, &QPushButton::clicked, this, &MainWindow::onBackToLoginClicked);

    connect(logoutButton, &QPushButton::clicked, this, &MainWindow::onLogoutClicked);
    connect(runButton, &QPushButton::clicked, this, &MainWindow::onRunClicked);

    connect(loadMapButton, &QPushButton::clicked, this, &MainWindow::onLoadMapClicked);
    connect(saveMapButton, &QPushButton::clicked, this, &MainWindow::onSaveMapClicked);
    connect(addVertexButton, &QPushButton::clicked, this, &MainWindow::onAddVertexClicked);
    connect(removeVertexButton, &QPushButton::clicked, this, &MainWindow::onRemoveVertexClicked);
    connect(addEdgeButton, &QPushButton::clicked, this, &MainWindow::onAddEdgeClicked);
    connect(removeEdgeButton, &QPushButton::clicked, this, &MainWindow::onRemoveEdgeClicked);
    connect(editEdgeButton, &QPushButton::clicked, this, &MainWindow::onEditEdgeClicked);

    connect(mapView, &MapView::nodeClicked, this, &MainWindow::onNodeClicked);
}

void MainWindow::showLoginPage() {
    stackedWidget->setCurrentIndex(0);
    setWindowTitle("地图导航系统 - 登录");
}

void MainWindow::showRegisterPage() {
    stackedWidget->setCurrentIndex(1);
    setWindowTitle("地图导航系统 - 注册");
}

void MainWindow::showMainPage() {
    stackedWidget->setCurrentIndex(2);
    applyRoleUi();
    outputEdit->setPlainText(QString("欢迎，%1。\n请先导入地图（管理员）或确保运行目录有 test.map。\n点击地图节点可自动选起点/终点。")
                                 .arg(currentUser));
}

void MainWindow::applyRoleUi() {
    bool isAdmin = (currentRole == UserRole::Admin);
    setWindowTitle(QString("地图导航系统 - %1（%2）").arg(currentUser, isAdmin ? "管理员" : "普通用户"));

    loadMapButton->setVisible(isAdmin);
    saveMapButton->setVisible(isAdmin);
    addVertexButton->setVisible(isAdmin);
    removeVertexButton->setVisible(isAdmin);
    addEdgeButton->setVisible(isAdmin);
    removeEdgeButton->setVisible(isAdmin);
    editEdgeButton->setVisible(isAdmin);
}

QString MainWindow::formatPath(const Path& p) const {
    if (p.nodes.empty()) return "(empty)";
    QString s;
    for (int i = 0; i < (int)p.nodes.size(); i++) {
        s += QString::number(p.nodes[i]);
        if (i + 1 < (int)p.nodes.size()) s += " -> ";
    }
    s += QString("    cost = %1").arg(p.cost);
    return s;
}

bool MainWindow::ensureGraphLoadedForUser() {
    if (graphLoaded) return true;

    graph.clear();
    if (loadMapFile("test.map", graph)) {
        graphLoaded = true;
        rebuildScene();
        return true;
    }

    outputEdit->setPlainText("未加载地图文件。\n管理员请点击【导入地图】加载 .map 文件。\n（默认会尝试读取运行目录的 test.map）");
    return false;
}

void MainWindow::rebuildScene() {
    scene->clear();
    edgeItems.clear();
    nodeItems.clear();
    highlightedShortest.clear();
    highlightedSecond.clear();

    selectedSourceId = -1;
    selectedTargetId = -1;
    nextClickFillSource = true;

    const auto& verts = graph.vertices();
    const int n = (int)verts.size();

    int edgeCount = 0;
    for (int u = 0; u < n; u++) {
        for (const Edge& e : graph.adjEdges(u)) {
            if (e.from < e.to) edgeCount++;
        }
    }
    mapInfoLabel->setText(QString("顶点：%1   道路：%2   （滚轮缩放，拖拽移动）").arg(n).arg(edgeCount));

    double minX = 1e18, minY = 1e18, maxX = -1e18, maxY = -1e18;
    for (int i = 0; i < n; i++) {
        const auto& v = verts[i];
        const double x = v.x();
        const double y = v.y();
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
    }

    QPen edgePen(QColor("#C9B27A"));
    edgePen.setWidthF(2.0);

    // draw edges
    for (int u = 0; u < n; u++) {
        for (const Edge& e : graph.adjEdges(u)) {
            int v = e.to;
            if (u >= v) continue;

            const auto& vu = verts[u];
            const auto& vv = verts[v];

            auto* line = scene->addLine(vu.x(), vu.y(), vv.x(), vv.y(), edgePen);
            line->setZValue(1);
            edgeItems.insert(edgeKey(u, v), line);
        }
    }

    // draw nodes
    const qreal r = 10.0;
    QBrush nodeBrush(QColor("#FFE08A"));
    QPen nodePen(QColor("#B0893E"));
    nodePen.setWidthF(2.0);

    for (int i = 0; i < n; i++) {
        const auto& v = verts[i];
        QString name = QString::fromStdString(v.name());
        if (name == "[removed]") continue;

        auto* ellipse = scene->addEllipse(v.x() - r, v.y() - r, 2*r, 2*r, nodePen, nodeBrush);
        ellipse->setZValue(10);
        ellipse->setData(0, i);
        ellipse->setToolTip(QString("ID: %1\nName: %2").arg(i).arg(name));
        nodeItems.insert(i, ellipse);

        auto* text = scene->addSimpleText(name);
        text->setPos(v.x() + 12, v.y() - 10);
        text->setZValue(11);
    }

    scene->setSceneRect(minX - 60, minY - 60, (maxX - minX) + 120, (maxY - minY) + 120);
    mapView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    mapView->scale(1.02, 1.02);
}

void MainWindow::clearPathHighlight() {
    QPen edgePen(QColor("#C9B27A"));
    edgePen.setWidthF(2.0);

    for (auto* item : highlightedShortest) if (item) item->setPen(edgePen);
    for (auto* item : highlightedSecond) if (item) item->setPen(edgePen);

    highlightedShortest.clear();
    highlightedSecond.clear();
}

void MainWindow::highlightPath(const Path& p, bool isShortest) {
    if (p.nodes.size() < 2) return;

    QPen pen = isShortest ? QPen(QColor("#D35400")) : QPen(QColor("#2E86C1"));
    pen.setWidthF(isShortest ? 5.0 : 4.0);
    if (!isShortest) pen.setStyle(Qt::DashLine);

    for (int i = 0; i + 1 < (int)p.nodes.size(); i++) {
        int u = p.nodes[i];
        int v = p.nodes[i + 1];
        auto it = edgeItems.find(edgeKey(u, v));
        if (it != edgeItems.end() && it.value()) {
            it.value()->setPen(pen);
            if (isShortest) highlightedShortest.push_back(it.value());
            else highlightedSecond.push_back(it.value());
        }
    }
}

void MainWindow::refreshNodeSelectionStyle() {
    // 默认样式
    QBrush normalBrush(QColor("#FFE08A"));
    QPen normalPen(QColor("#B0893E"));
    normalPen.setWidthF(2.0);

    // 起点：更深的橙色
    QBrush sourceBrush(QColor("#FFB347"));
    QPen sourcePen(QColor("#C65A00"));
    sourcePen.setWidthF(3.0);

    // 终点：蓝色
    QBrush targetBrush(QColor("#7FB3D5"));
    QPen targetPen(QColor("#1F618D"));
    targetPen.setWidthF(3.0);

    // 先全恢复
    for (auto it = nodeItems.begin(); it != nodeItems.end(); ++it) {
        if (it.value()) {
            it.value()->setBrush(normalBrush);
            it.value()->setPen(normalPen);
        }
    }

    // 再给起点/终点上色
    if (nodeItems.contains(selectedSourceId) && nodeItems[selectedSourceId]) {
        nodeItems[selectedSourceId]->setBrush(sourceBrush);
        nodeItems[selectedSourceId]->setPen(sourcePen);
    }
    if (nodeItems.contains(selectedTargetId) && nodeItems[selectedTargetId]) {
        nodeItems[selectedTargetId]->setBrush(targetBrush);
        nodeItems[selectedTargetId]->setPen(targetPen);
    }
}

// ===== click node: fill source then target =====
void MainWindow::onNodeClicked(int nodeId) {
    if (!graphLoaded) {
        outputEdit->setPlainText("请先加载地图后再点击节点。");
        return;
    }

    if (nextClickFillSource) {
        selectedSourceId = nodeId;
        sourceEdit->setText(QString::number(nodeId));
        nextClickFillSource = false;
        outputEdit->setPlainText(QString("已选择起点：%1\n（再点击一个节点选择终点）").arg(nodeId));
    } else {
        selectedTargetId = nodeId;
        targetEdit->setText(QString::number(nodeId));
        nextClickFillSource = true;
        outputEdit->setPlainText(QString("已选择终点：%1\n（再点击节点将重新选择起点）").arg(nodeId));
    }

    refreshNodeSelectionStyle();
}

// ===== page actions =====
void MainWindow::onGoRegisterClicked() {
    regUserEdit->setText(loginUserEdit->text().trimmed());
    regPassEdit->clear();
    regPass2Edit->clear();
    showRegisterPage();
}

void MainWindow::onBackToLoginClicked() {
    showLoginPage();
}

// ===== register =====
void MainWindow::onDoRegisterClicked() {
    QString username = regUserEdit->text().trimmed();
    QString pass1 = regPassEdit->text();
    QString pass2 = regPass2Edit->text();

    if (pass1 != pass2) {
        QMessageBox::warning(this, "注册失败", "两次输入的密码不一致");
        return;
    }

    QString err;
    if (!userManager.registerUser(username, pass1, err)) {
        QMessageBox::warning(this, "注册失败", err);
        return;
    }

    if (!userManager.save(kUserFile)) {
        QMessageBox::warning(this, "提示", "注册成功，但保存用户文件失败");
    } else {
        QMessageBox::information(this, "注册成功", "注册成功，请返回登录");
        showLoginPage();
    }
}

// ===== login =====
void MainWindow::onLoginClicked() {
    QString username = loginUserEdit->text().trimmed();
    QString password = loginPassEdit->text();
    UserRole role = adminRadio->isChecked() ? UserRole::Admin : UserRole::User;

    QString err;
    if (!userManager.login(username, password, role, err)) {
        QMessageBox::warning(this, "登录失败", err);
        return;
    }

    currentUser = username;
    currentRole = role;

    showMainPage();
    ensureGraphLoadedForUser();
}

void MainWindow::onLogoutClicked() {
    currentUser.clear();
    currentRole = UserRole::User;

    graph.clear();
    graphLoaded = false;
    scene->clear();
    edgeItems.clear();
    nodeItems.clear();
    mapInfoLabel->setText("未加载地图");

    loginPassEdit->clear();
    showLoginPage();
}

// ===== navigation =====
void MainWindow::onRunClicked() {
    bool ok1 = false, ok2 = false;
    int source = sourceEdit->text().toInt(&ok1);
    int target = targetEdit->text().toInt(&ok2);

    if (!ok1 || !ok2) {
        QMessageBox::warning(this, "输入错误", "请输入合法的起点/终点ID（整数）");
        return;
    }

    if (!ensureGraphLoadedForUser()) return;

    Path shortest;
    Path secondShortest;
    bool hasPath = yenK2(graph, source, target, shortest, secondShortest);

    clearPathHighlight();

    if (!hasPath) {
        outputEdit->setPlainText(QString("找不到从 %1 到 %2 的路径。").arg(source).arg(target));
        return;
    }

    highlightPath(shortest, true);
    highlightPath(secondShortest, false);

    QString out;
    out += "Shortest path:\n";
    out += formatPath(shortest);
    out += "\n\nSecond shortest path:\n";
    out += formatPath(secondShortest);
    out += "\n\n（地图中已高亮：最短=橙色实线，次短=蓝色虚线）";
    outputEdit->setPlainText(out);
}

// ===== admin map management =====
void MainWindow::onLoadMapClicked() {
    if (currentRole != UserRole::Admin) return;

    QString fileName = QFileDialog::getOpenFileName(this, "选择地图文件", "", "Map Files (*.map);;All Files (*)");
    if (fileName.isEmpty()) return;

    graph.clear();
    if (!loadMapFile(fileName.toStdString(), graph)) {
        QMessageBox::warning(this, "导入失败", "地图文件读取失败或格式错误");
        graphLoaded = false;
        return;
    }

    graphLoaded = true;
    rebuildScene();
    outputEdit->setPlainText(QString("导入成功：%1\n顶点数：%2").arg(fileName).arg(graph.vertexCount()));
}

void MainWindow::onSaveMapClicked() {
    if (currentRole != UserRole::Admin) return;
    if (!graphLoaded) {
        QMessageBox::warning(this, "提示", "当前没有加载地图，无法保存");
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this, "保存地图文件", "", "Map Files (*.map);;All Files (*)");
    if (fileName.isEmpty()) return;

    if (!saveMapFile(fileName.toStdString(), graph)) {
        QMessageBox::warning(this, "保存失败", "保存地图文件失败");
        return;
    }

    outputEdit->setPlainText(QString("保存成功：%1").arg(fileName));
}

void MainWindow::onAddVertexClicked() {
    if (currentRole != UserRole::Admin) return;

    bool ok = false;
    QString name = QInputDialog::getText(this, "新增地区", "地区名称：", QLineEdit::Normal, "", &ok);
    if (!ok || name.trimmed().isEmpty()) return;

    double x = QInputDialog::getDouble(this, "新增地区", "x 坐标：", 100.0, -1e9, 1e9, 2, &ok);
    if (!ok) return;
    double y = QInputDialog::getDouble(this, "新增地区", "y 坐标：", 100.0, -1e9, 1e9, 2, &ok);
    if (!ok) return;

    int id = graph.addVertex(name.toStdString(), x, y);
    graphLoaded = true;
    rebuildScene();
    outputEdit->setPlainText(QString("新增地区成功：id=%1, name=%2").arg(id).arg(name));
}

void MainWindow::onRemoveVertexClicked() {
    if (currentRole != UserRole::Admin) return;
    if (!graphLoaded) {
        QMessageBox::warning(this, "提示", "未加载地图");
        return;
    }

    bool ok = false;
    int id = QInputDialog::getInt(this, "删除地区", "地区ID：", 0, 0, 1000000, 1, &ok);
    if (!ok) return;

    if (!graph.removeVertex(id)) {
        QMessageBox::warning(this, "失败", "删除失败：ID 不存在或已删除");
        return;
    }

    rebuildScene();
    outputEdit->setPlainText(QString("删除地区成功：id=%1").arg(id));
}

void MainWindow::onAddEdgeClicked() {
    if (currentRole != UserRole::Admin) return;
    if (!graphLoaded) {
        QMessageBox::warning(this, "提示", "未加载地图（可先新增地区或导入地图）");
        return;
    }

    bool ok = false;
    int u = QInputDialog::getInt(this, "新增道路", "起点ID u：", 0, 0, 1000000, 1, &ok);
    if (!ok) return;
    int v = QInputDialog::getInt(this, "新增道路", "终点ID v：", 0, 0, 1000000, 1, &ok);
    if (!ok) return;
    double w = QInputDialog::getDouble(this, "新增道路", "道路长度/权值 w：", 1.0, 0.0, 1e18, 2, &ok);
    if (!ok) return;

    if (!graph.addEdge(u, v, w)) {
        QMessageBox::warning(this, "失败", "新增道路失败：检查ID是否存在、u!=v 等");
        return;
    }

    rebuildScene();
    outputEdit->setPlainText(QString("新增/更新道路成功：%1 <-> %2, w=%3").arg(u).arg(v).arg(w));
}

void MainWindow::onRemoveEdgeClicked() {
    if (currentRole != UserRole::Admin) return;
    if (!graphLoaded) {
        QMessageBox::warning(this, "提示", "未加载地图");
        return;
    }

    bool ok = false;
    int u = QInputDialog::getInt(this, "删除道路", "起点ID u：", 0, 0, 1000000, 1, &ok);
    if (!ok) return;
    int v = QInputDialog::getInt(this, "删除道路", "终点ID v：", 0, 0, 1000000, 1, &ok);
    if (!ok) return;

    if (!graph.removeEdge(u, v)) {
        QMessageBox::warning(this, "失败", "删除道路失败：该道路不存在");
        return;
    }

    rebuildScene();
    outputEdit->setPlainText(QString("删除道路成功：%1 <-> %2").arg(u).arg(v));
}

void MainWindow::onEditEdgeClicked() {
    if (currentRole != UserRole::Admin) return;
    if (!graphLoaded) {
        QMessageBox::warning(this, "提示", "未加载地图");
        return;
    }

    bool ok = false;
    int u = QInputDialog::getInt(this, "修改道路长度", "起点ID u：", 0, 0, 1000000, 1, &ok);
    if (!ok) return;
    int v = QInputDialog::getInt(this, "修改道路长度", "终点ID v：", 0, 0, 1000000, 1, &ok);
    if (!ok) return;

    if (!graph.hasEdge(u, v)) {
        QMessageBox::warning(this, "失败", "该道路不存在，无法修改");
        return;
    }

    double w = QInputDialog::getDouble(this, "修改道路长度", "新长度/权值 w：", 1.0, 0.0, 1e18, 2, &ok);
    if (!ok) return;

    graph.addEdge(u, v, w);
    rebuildScene();
    outputEdit->setPlainText(QString("修改道路成功：%1 <-> %2, w=%3").arg(u).arg(v).arg(w));
}
