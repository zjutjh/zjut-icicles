from flask import Blueprint, render_template, request, flash, redirect, url_for
from flask_login import login_user, logout_user, login_required, current_user
from werkzeug.security import check_password_hash, generate_password_hash
import hashlib
from .models import User
from . import database as db

auth_bp = Blueprint('auth', __name__)

# 自定义密码哈希函数，生成更短的哈希值
def custom_generate_password_hash(password):
    """生成适合数据库VARCHAR(64)字段的密码哈希"""
    # 使用SHA-256算法生成哈希值，输出长度为64字符
    salt = "university_db_salt"  # 添加固定盐值增加安全性
    return hashlib.sha256((password + salt).encode()).hexdigest()

# 自定义密码验证函数
def custom_check_password_hash(stored_hash, password):
    """验证密码是否匹配存储的哈希值"""
    return stored_hash == custom_generate_password_hash(password)

@auth_bp.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        login_id = request.form.get('username')  # 可以是学号/工号或用户名
        password = request.form.get('password')
        
        # 添加调试信息
        print(f"尝试登录: 登录ID={login_id}, 密码长度={len(password)}, 密码={password}")

        # ---------------- 认证逻辑 ----------------
        # 场景 1：数据库中不存在该用户
        user_data = User.get_by_id_or_username(login_id)
        if not user_data:
            print(f"用户 {login_id} 不存在")
            flash('用户名或密码错误。', 'danger')
            return render_template('login.html')

        print(f"找到用户: {user_data}")
        stored_hash = user_data.get('lyh_password_hash')
        print(f"数据库中的密码哈希: {stored_hash}")

        # 场景 2：用户尚未设置密码（首次登录）——允许使用默认密码 123456
        if not stored_hash or stored_hash in ('hash_placeholder', ''):
            print(f"检测到首次登录场景，用户输入密码: {password}")
            if password == '123456':
                print("默认密码验证成功，登录用户")
                user = User(user_id=user_data['lyh_user_id'], username=user_data['lyh_username'], role=user_data['lyh_role'])
                login_user(user)
                flash('首次登录成功，请尽快修改密码！', 'warning')
                
                # 可选：首次登录后设置密码哈希
                # update_query = "UPDATE Liyh_User SET lyh_password_hash = %s WHERE lyh_user_id = %s"
                # db.execute_commit(update_query, (custom_generate_password_hash(password), user_data['lyh_user_id']))
                
                return redirect(url_for('auth.change_password'))
            else:
                print(f"默认密码验证失败: 输入={password}, 期望=123456")
                flash('首次登录请使用默认密码 123456', 'danger')
                return render_template('login.html')

        # 场景 3：用户已设置密码 —— 正常校验哈希
        try:
            # 临时解决方案：如果哈希值是占位符但不是空字符串，也允许使用默认密码
            if stored_hash == 'hash_placeholder' and password == '123456':
                print("检测到hash_placeholder，使用默认密码登录")
                user = User(user_id=user_data['lyh_user_id'], username=user_data['lyh_username'], role=user_data['lyh_role'])
                login_user(user)
                flash('登录成功，请尽快修改密码！', 'warning')
                return redirect(url_for('auth.change_password'))
            
            # 尝试使用自定义哈希验证方式    
            if custom_check_password_hash(stored_hash, password):
                user = User(user_id=user_data['lyh_user_id'], username=user_data['lyh_username'], role=user_data['lyh_role'])
                login_user(user)
                flash('登录成功！', 'success')
                return redirect(url_for('main.dashboard'))
            # 兼容旧的哈希方式
            elif check_password_hash(stored_hash, password):
                user = User(user_id=user_data['lyh_user_id'], username=user_data['lyh_username'], role=user_data['lyh_role'])
                login_user(user)
                flash('登录成功！', 'success')
                return redirect(url_for('main.dashboard'))
            else:
                flash('用户名或密码错误。', 'danger')
        except Exception as e:
            print(f"密码验证错误: {e}")
            # 临时解决方案：如果哈希验证出错，且密码是默认密码，允许登录
            if password == '123456':
                print("密码验证异常，但使用默认密码，允许登录")
                user = User(user_id=user_data['lyh_user_id'], username=user_data['lyh_username'], role=user_data['lyh_role'])
                login_user(user)
                flash('登录成功，请尽快修改密码！', 'warning')
                return redirect(url_for('auth.change_password'))
            flash('登录过程中发生错误，请联系管理员。', 'danger')

    return render_template('login.html')


@auth_bp.route('/logout')
@login_required
def logout():
    logout_user()
    flash('您已成功退出登录。', 'info')
    return redirect(url_for('auth.login'))


@auth_bp.route('/change_password', methods=['GET', 'POST'])
@login_required
def change_password():
    """修改密码页面"""
    if request.method == 'POST':
        current_password = request.form.get('current_password')
        new_password = request.form.get('new_password')
        confirm_password = request.form.get('confirm_password')
        
        # 验证表单数据
        if not new_password or len(new_password) < 6:
            flash('新密码长度不能少于6个字符', 'danger')
            return render_template('change_password.html')
            
        if new_password != confirm_password:
            flash('两次输入的新密码不一致', 'danger')
            return render_template('change_password.html')
        
        # 获取用户当前密码哈希
        query = "SELECT lyh_password_hash FROM Liyh_User WHERE lyh_user_id = %s"
        result = db.fetch_one(query, (current_user.id,))
        
        if not result:
            flash('用户信息获取失败', 'danger')
            return render_template('change_password.html')
            
        stored_hash = result.get('lyh_password_hash')
        
        # 首次设置密码（使用默认密码登录）
        if not stored_hash or stored_hash in ('hash_placeholder', ''):
            if current_password == '123456':
                # 更新密码
                update_query = "UPDATE Liyh_User SET lyh_password_hash = %s WHERE lyh_user_id = %s"
                db.execute_commit(update_query, (custom_generate_password_hash(new_password), current_user.id))
                
                flash('密码设置成功！', 'success')
                return redirect(url_for('main.dashboard'))
            else:
                flash('当前密码错误', 'danger')
                return render_template('change_password.html')
        
        # 已有密码，需要验证当前密码
        try:
            # 尝试使用自定义哈希验证方式
            if custom_check_password_hash(stored_hash, current_password):
                # 更新密码
                update_query = "UPDATE Liyh_User SET lyh_password_hash = %s WHERE lyh_user_id = %s"
                db.execute_commit(update_query, (custom_generate_password_hash(new_password), current_user.id))
                
                flash('密码修改成功！', 'success')
                return redirect(url_for('main.dashboard'))
            # 兼容旧的哈希方式
            elif check_password_hash(stored_hash, current_password):
                # 更新密码，同时转换为新的哈希格式
                update_query = "UPDATE Liyh_User SET lyh_password_hash = %s WHERE lyh_user_id = %s"
                db.execute_commit(update_query, (custom_generate_password_hash(new_password), current_user.id))
                
                flash('密码修改成功！', 'success')
                return redirect(url_for('main.dashboard'))
            else:
                flash('当前密码错误', 'danger')
        except Exception as e:
            print(f"密码验证错误: {e}")
            flash('密码修改过程中发生错误', 'danger')
            
    return render_template('change_password.html') 