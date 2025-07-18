from flask import Flask
from flask_login import LoginManager
from .config import get_config
import os

# 初始化Flask-Login
login_manager = LoginManager()

def create_app(config_name=None):
    """应用工厂函数"""
    app = Flask(__name__)
    
    # 加载配置
    if config_name is None:
        config_class = get_config()
    else:
        from .config import config
        config_class = config.get(config_name, get_config())
    
    app.config.from_object(config_class)
    
    # 初始化扩展
    login_manager.init_app(app)
    login_manager.login_view = 'auth.login'
    login_manager.login_message = '请先登录以访问此页面。'
    login_manager.login_message_category = 'info'
    
    # 用户加载回调
    @login_manager.user_loader
    def load_user(user_id):
        from .models import User
        return User.get(user_id)
    
    # 注册蓝图
    from .main import main_bp
    from .auth import auth_bp
    
    app.register_blueprint(main_bp)
    app.register_blueprint(auth_bp, url_prefix='/auth')
    
    # 错误处理
    @app.errorhandler(404)
    def not_found_error(error):
        from flask import render_template
        return render_template('errors/404.html'), 404
    
    @app.errorhandler(500)
    def internal_error(error):
        from flask import render_template
        return render_template('errors/500.html'), 500
    
    @app.errorhandler(403)
    def forbidden_error(error):
        from flask import render_template
        return render_template('errors/403.html'), 403
    
    # 上下文处理器
    @app.context_processor
    def inject_config():
        """向模板注入配置变量"""
        return {
            'SYSTEM_NAME': app.config.get('SYSTEM_NAME', '教务管理系统'),
            'SYSTEM_VERSION': app.config.get('SYSTEM_VERSION', '1.0.0'),
            'CURRENT_SEMESTER': app.config.get('CURRENT_SEMESTER', '2024-Fall')
        }
    
    # 创建上传目录
    upload_folder = app.config.get('UPLOAD_FOLDER')
    if upload_folder and not os.path.exists(upload_folder):
        os.makedirs(upload_folder)
    
    return app 