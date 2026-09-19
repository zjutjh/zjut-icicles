#pragma once
#include <QString>
#include <QVector>

enum class UserRole {
    User = 0,
    Admin = 1
};

struct UserRecord {
    QString username;
    QString password;
    UserRole role = UserRole::User;
};

class UserManager {
public:
    bool load(const QString& fileName);
    bool save(const QString& fileName) const;

    // 系统默认管理员 admin/admin（如果不存在就写入）
    void ensureDefaultAdmin(const QString& fileName);

    // 只允许注册普通用户（不允许注册管理员，不允许注册 admin）
    bool registerUser(const QString& username,
                      const QString& password,
                      QString& err);

    // 登录时需要选择身份（管理员/普通用户）
    bool login(const QString& username,
               const QString& password,
               UserRole role,
               QString& err) const;

private:
    QVector<UserRecord> users;

    int findUserIndex(const QString& username) const;
};
