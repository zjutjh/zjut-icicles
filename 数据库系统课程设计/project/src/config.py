import os
from datetime import timedelta

class Config:
    """基础配置类"""
    
    # Flask基础配置
    SECRET_KEY = os.environ.get('SECRET_KEY') or 'your-secret-key-change-in-production'
    
    # 数据库配置
    DATABASE_CONFIG = {
        'host': '110.41.119.192',
        'port': '8000',
        'database': 'db_zjut',
        'user': 'db_user35',
        'password': 'db_user35@123'
    }
    
    # Flask-Login配置
    REMEMBER_COOKIE_DURATION = timedelta(days=7)
    SESSION_PROTECTION = 'strong'
    
    # 应用配置
    ITEMS_PER_PAGE = 20  # 分页每页显示条数
    SEARCH_RESULTS_LIMIT = 50  # 搜索结果限制
    
    # 文件上传配置
    MAX_CONTENT_LENGTH = 16 * 1024 * 1024  # 16MB
    UPLOAD_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'uploads')
    
    # 系统配置
    SYSTEM_NAME = '教务管理系统'
    SYSTEM_VERSION = '1.0.0'
    
    # 当前学期配置（可以从数据库动态获取）
    CURRENT_SEMESTER = '2024-Fall'
    
    # 时间配置
    TIMEZONE = 'Asia/Shanghai'
    
class DevelopmentConfig(Config):
    """开发环境配置"""
    DEBUG = True
    TESTING = False
    
class ProductionConfig(Config):
    """生产环境配置"""
    DEBUG = False
    TESTING = False
    
    # 生产环境下的安全配置
    SESSION_COOKIE_SECURE = True
    SESSION_COOKIE_HTTPONLY = True
    SESSION_COOKIE_SAMESITE = 'Lax'
    
class TestingConfig(Config):
    """测试环境配置"""
    DEBUG = True
    TESTING = True
    
    # 测试数据库配置
    DATABASE_CONFIG = {
        'host': 'localhost',
        'port': 5432,
        'database': 'test_university_db',
        'user': 'postgres',
        'password': 'password'
    }

# 配置字典
config = {
    'development': DevelopmentConfig,
    'production': ProductionConfig,
    'testing': TestingConfig,
    'default': DevelopmentConfig
}

def get_config():
    """获取当前配置"""
    config_name = os.environ.get('FLASK_ENV') or 'default'
    return config.get(config_name, DevelopmentConfig) 