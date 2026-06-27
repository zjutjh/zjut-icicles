#include "UserManager.h"
#include <QFile>
#include <QTextStream>

// 根据用户名查找用户索引，未找到返回-1
int UserManager::findUserIndex(const QString& username) const {
    for (int i = 0; i < users.size(); ++i) {
        if (users[i].username == username) return i;
    }
    return -1;
}

// 从文件中会载用户信息
bool UserManager::load(const QString& fileName) {
    users.clear();

    QFile f(fileName);
    if (!f.exists()) return true;
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

    QTextStream in(&f);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        QStringList parts = line.split(' ', Qt::SkipEmptyParts);
        if (parts.size() < 3) continue;

        UserRecord u;
        u.username = parts[0];
        u.password = parts[1];
        u.role = (parts[2].toLower() == "admin") ? UserRole::Admin : UserRole::User;
        users.push_back(u);
    }
    return true;
}

// 将用户信息保存到文件
bool UserManager::save(const QString& fileName) const {
    QFile f(fileName);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return false;

    QTextStream out(&f);
    out << "# 用户名 密码 消息(user/admin)\n";
    for (const auto& u : users) {
        out << u.username << " " << u.password << " " << (u.role == UserRole::Admin ? "admin" : "user") << "\n";
    }
    return true;
}

// 确保系统中存在admin账户，不存在则创建
void UserManager::ensureDefaultAdmin(const QString& fileName) {
    if (findUserIndex("admin") != -1) return;
    users.push_back({"admin", "admin", UserRole::Admin});
    save(fileName);
}

// 注册新用户（普通用户）
bool UserManager::registerUser(const QString& username,
                               const QString& password,
                               QString& err) {
    QString u = username.trimmed();
    if (u.isEmpty() || password.isEmpty()) {
        err = "用户名或密码不能为空";
        return false;
    }

    // 验证用户名长度（1-32 字符）
    if (u.length() < 1 || u.length() > 32) {
        err = "用户名长度必须在 1-32 字符之间";
        return false;
    }

    // 验证密码长度（6-128 字符）
    if (password.length() < 6 || password.length() > 128) {
        err = "密码长度必须在 6-128 字符之间";
        return false;
    }

    if (u.toLower() == "admin") {
        err = "admin 为系统默认管理员账号，不能注册";
        return false;
    }
    if (findUserIndex(u) != -1) {
        err = "用户名已存在";
        return false;
    }

    // 只注册普通用户
    users.push_back({u, password, UserRole::User});
    err.clear();
    return true;
}

// 用户登录验证
bool UserManager::login(const QString& username,
                        const QString& password,
                        UserRole role,
                        QString& err) const {
    QString u = username.trimmed();
    int idx = findUserIndex(u);
    if (idx == -1) {
        err = "用户不存在";
        return false;
    }
    if (users[idx].password != password) {
        err = "密码错误";
        return false;
    }
    if (users[idx].role != role) {
        err = "身份不匹配（管理员/普通用户选择错误）";
        return false;
    }
    err.clear();
    return true;
}
