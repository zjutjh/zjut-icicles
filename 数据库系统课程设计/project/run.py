#!/usr/bin/env python3
"""
教务管理系统启动文件

运行此文件来启动Flask开发服务器。
在生产环境中，请使用WSGI服务器如gunicorn。

使用方法:
    python run.py
    
环境变量:
    FLASK_ENV: 设置为 'development', 'production' 或 'testing'
    DB_HOST: 数据库主机地址
    DB_PORT: 数据库端口
    DB_NAME: 数据库名称
    DB_USER: 数据库用户名
    DB_PASSWORD: 数据库密码
"""

import os
import sys
from src import create_app
from src.database import test_connection, cleanup_database
from src.config import get_config

def main():
    """主函数"""
    try:
        # 创建Flask应用
        app = create_app()
        
        # 测试数据库连接
        print("正在测试数据库连接...")
        if not test_connection():
            print("❌ 数据库连接失败！请检查配置。")
            print("请确保：")
            print("1. PostgreSQL服务正在运行")
            print("2. 数据库已创建并执行了初始化脚本")
            print("3. 数据库连接参数正确")
            sys.exit(1)
        
        print("✅ 数据库连接成功！")
        
        # 获取配置
        config = get_config()
        
        # 显示启动信息
        print(f"\n🚀 {config.SYSTEM_NAME} v{config.SYSTEM_VERSION}")
        print(f"📊 当前学期: {config.CURRENT_SEMESTER}")
        print(f"🔧 运行模式: {'开发模式' if config.DEBUG else '生产模式'}")
        print(f"🌐 访问地址: http://localhost:5000")
        print("\n用户角色说明:")
        print("  👨‍🎓 学生: 使用学号登录 (例如: 2023001)")
        print("  👨‍🏫 教师: 使用工号登录 (例如: T001)")
        print("  👨‍💼 管理员: 使用管理员ID登录 (例如: A001)")
        print("  🔑 密码: 演示版本中可使用任意密码")
        print("\n按 Ctrl+C 停止服务器")
        print("-" * 50)
        
        # 启动Flask开发服务器
        try:
            app.run(
                host='0.0.0.0',
                port=5000,
                debug=config.DEBUG,
                threaded=True
            )
        except KeyboardInterrupt:
            print("\n\n👋 服务器已停止")
        finally:
            # 清理数据库连接池
            cleanup_database()
            
    except Exception as e:
        print(f"❌ 启动失败: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main() 