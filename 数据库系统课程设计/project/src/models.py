from flask_login import UserMixin
from . import database as db

class User(UserMixin):
    def __init__(self, user_id, username, role):
        self.id = user_id
        self.username = username
        self.role = role

    @staticmethod
    def get(user_id):
        """
        根据用户ID从数据库获取用户信息，并创建一个User对象。
        这是 Flask-Login 要求的回调函数。
        """
        query = "SELECT lyh_user_id, lyh_username, lyh_role FROM Liyh_User WHERE lyh_user_id = %s"
        result = db.fetch_one(query, (user_id,))
        if result:
            return User(user_id=result['lyh_user_id'], username=result['lyh_username'], role=result['lyh_role'])
        return None

    @staticmethod
    def get_by_username(username):
        """根据用户名获取用户信息"""
        query = "SELECT lyh_user_id, lyh_username, lyh_role, lyh_password_hash FROM Liyh_User WHERE lyh_username = %s"
        return db.fetch_one(query, (username,))
        
    @staticmethod
    def get_by_id_or_username(login_id):
        """根据ID或用户名获取用户信息（支持多种登录方式）"""
        # 先尝试按用户ID查询
        query1 = "SELECT lyh_user_id, lyh_username, lyh_role, lyh_password_hash FROM Liyh_User WHERE lyh_user_id = %s"
        result = db.fetch_one(query1, (login_id,))
        
        if result:
            return result
            
        # 如果找不到，再尝试按用户名查询
        query2 = "SELECT lyh_user_id, lyh_username, lyh_role, lyh_password_hash FROM Liyh_User WHERE lyh_username = %s"
        return db.fetch_one(query2, (login_id,)) 