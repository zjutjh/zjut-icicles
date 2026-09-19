#pragma once
#include <QMainWindow>
#include <QString>
#include <QHash>
#include <QList>

#include "Graph.h"
#include "Yen.h"
#include "FileIO.h"
#include "UserManager.h"

class QStackedWidget;
class QLineEdit;
class QTextEdit;
class QPushButton;
class QRadioButton;
class QLabel;
class QGraphicsScene;
class QGraphicsLineItem;
class QGraphicsEllipseItem;

class MapView;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    // 页面切换
    void onGoRegisterClicked();
    void onBackToLoginClicked();

    // 登录/注册
    void onLoginClicked();
    void onDoRegisterClicked();
    void onLogoutClicked();

    // 导航
    void onRunClicked();

    // 管理员地图管理
    void onLoadMapClicked();
    void onSaveMapClicked();
    void onAddVertexClicked();
    void onRemoveVertexClicked();
    void onAddEdgeClicked();
    void onRemoveEdgeClicked();
    void onEditEdgeClicked();

    // 可视化交互：点击节点
    void onNodeClicked(int nodeId);

private:
    void buildUi();
    void applyTheme();
    void showLoginPage();
    void showRegisterPage();
    void showMainPage();
    void applyRoleUi();

    bool ensureGraphLoadedForUser();

    void rebuildScene();
    void clearPathHighlight();
    void highlightPath(const Path& p, bool isShortest);

    void refreshNodeSelectionStyle();

    QString formatPath(const Path& p) const;
    long long edgeKey(int u, int v) const;

private:
    // ===== state =====
    UserManager userManager;
    QString currentUser;
    UserRole currentRole = UserRole::User;

    Graph graph;
    bool graphLoaded = false;

    // 点击选点：先填起点后填终点
    bool nextClickFillSource = true;
    int selectedSourceId = -1;
    int selectedTargetId = -1;

    // ===== UI =====
    QStackedWidget* stackedWidget = nullptr;

    // Login Page
    QLineEdit* loginUserEdit = nullptr;
    QLineEdit* loginPassEdit = nullptr;
    QRadioButton* adminRadio = nullptr;
    QRadioButton* userRadio = nullptr;
    QPushButton* loginButton = nullptr;
    QPushButton* goRegisterButton = nullptr;

    // Register Page
    QLineEdit* regUserEdit = nullptr;
    QLineEdit* regPassEdit = nullptr;
    QLineEdit* regPass2Edit = nullptr;
    QPushButton* doRegisterButton = nullptr;
    QPushButton* backToLoginButton = nullptr;

    // Main Page (Map + Panel)
    MapView* mapView = nullptr;
    QGraphicsScene* scene = nullptr;

    QLabel* mapInfoLabel = nullptr;

    QLineEdit* sourceEdit = nullptr;
    QLineEdit* targetEdit = nullptr;
    QPushButton* runButton = nullptr;
    QTextEdit* outputEdit = nullptr;
    QPushButton* logoutButton = nullptr;

    // Admin buttons
    QPushButton* loadMapButton = nullptr;
    QPushButton* saveMapButton = nullptr;
    QPushButton* addVertexButton = nullptr;
    QPushButton* removeVertexButton = nullptr;
    QPushButton* addEdgeButton = nullptr;
    QPushButton* removeEdgeButton = nullptr;
    QPushButton* editEdgeButton = nullptr;

    // ===== visualization caches =====
    // 无向边：u<v 的 key
    QHash<long long, QGraphicsLineItem*> edgeItems;

    // 节点：id -> 圆点 item
    QHash<int, QGraphicsEllipseItem*> nodeItems;

    QList<QGraphicsLineItem*> highlightedShortest;
    QList<QGraphicsLineItem*> highlightedSecond;
};
