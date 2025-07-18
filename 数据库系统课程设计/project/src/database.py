import psycopg2
from psycopg2.extras import RealDictCursor
from psycopg2 import pool
import threading
from flask import current_app
from .config import get_config

# 全局连接池
_connection_pool = None
_pool_lock = threading.Lock()

def get_connection_pool():
    """获取连接池，如果不存在则创建"""
    global _connection_pool
    
    if _connection_pool is None:
        with _pool_lock:
            if _connection_pool is None:
                config = get_config()
                db_config = config.DATABASE_CONFIG
                
                try:
                    _connection_pool = psycopg2.pool.ThreadedConnectionPool(
                        minconn=1,
                        maxconn=20,
                        host=db_config['host'],
                        port=db_config['port'],
                        database=db_config['database'],
                        user=db_config['user'],
                        password=db_config['password']
                    )
                    print("数据库连接池创建成功")
                except Exception as e:
                    print(f"创建数据库连接池失败: {e}")
                    raise
    
    return _connection_pool

def get_db_connection():
    """从连接池获取数据库连接"""
    try:
        pool = get_connection_pool()
        if pool:
            return pool.getconn()
        else:
            raise Exception("连接池未初始化")
    except Exception as e:
        print(f"获取数据库连接失败: {e}")
        raise

def return_db_connection(conn):
    """将连接返回到连接池"""
    try:
        pool = get_connection_pool()
        if pool and conn:
            pool.putconn(conn)
    except Exception as e:
        print(f"返回数据库连接失败: {e}")

def close_connection_pool():
    """关闭连接池"""
    global _connection_pool
    if _connection_pool:
        _connection_pool.closeall()
        _connection_pool = None
        print("数据库连接池已关闭")

def execute_query(query, params=None, fetch_one=False, fetch_all=True):
    """
    执行查询并返回结果
    
    Args:
        query: SQL查询语句
        params: 查询参数
        fetch_one: 是否只获取一行结果
        fetch_all: 是否获取所有结果
    
    Returns:
        查询结果
    """
    conn = None
    cursor = None
    
    try:
        conn = get_db_connection()
        cursor = conn.cursor(cursor_factory=RealDictCursor)
        
        # 处理参数，确保安全执行
        if params is not None:
            # 如果是空元组，将其转换为None
            if isinstance(params, tuple) and len(params) == 0:
                params = None
                
            # 如果是单个值，确保它是元组形式
            if not isinstance(params, (list, tuple, dict)) and params is not None:
                params = (params,)
                
            # 打印参数信息，便于调试
            print(f"执行查询参数类型: {type(params)}, 值: {params}")
            
            cursor.execute(query, params)
        else:
            cursor.execute(query)
        
        if fetch_one:
            result = cursor.fetchone()
            return dict(result) if result else None
        elif fetch_all:
            results = cursor.fetchall()
            return [dict(row) for row in results]
        else:
            return None
            
    except Exception as e:
        print(f"执行查询失败: {e}")
        print(f"查询语句: {query}")
        print(f"参数: {params}")
        raise
    finally:
        if cursor:
            cursor.close()
        if conn:
            return_db_connection(conn)

def execute_commit(query, params=None):
    """
    执行修改操作（INSERT, UPDATE, DELETE）并提交
    
    Args:
        query: SQL语句
        params: 参数
    
    Returns:
        影响的行数
    """
    conn = None
    cursor = None
    
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        cursor.execute(query, params)
        rowcount = cursor.rowcount
        conn.commit()
        
        return rowcount
        
    except Exception as e:
        if conn:
            conn.rollback()
        print(f"执行修改操作失败: {e}")
        print(f"查询语句: {query}")
        print(f"参数: {params}")
        raise
    finally:
        if cursor:
            cursor.close()
        if conn:
            return_db_connection(conn)

def fetch_one(query, params=None):
    """获取单行结果"""
    return execute_query(query, params, fetch_one=True, fetch_all=False)

def fetch_all(query, params=None):
    """获取所有结果"""
    return execute_query(query, params, fetch_one=False, fetch_all=True)

def test_connection():
    """测试数据库连接"""
    try:
        result = fetch_one("SELECT 1 as test")
        if result and result.get('test') == 1:
            print("数据库连接测试成功")
            return True
        else:
            print("数据库连接测试失败")
            return False
    except Exception as e:
        print(f"数据库连接测试失败: {e}")
        return False

def execute_transaction(queries_and_params):
    """
    在单个事务中执行多个SQL语句
    
    Args:
        queries_and_params: 包含多个(query, params)元组的列表
    
    Returns:
        最后一个查询的结果行数
    """
    conn = None
    cursor = None
    
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        # 开始事务
        conn.autocommit = False
        
        last_rowcount = 0
        
        # 执行每个查询
        for query, params in queries_and_params:
            cursor.execute(query, params)
            last_rowcount = cursor.rowcount
        
        # 提交事务
        conn.commit()
        
        return last_rowcount
        
    except Exception as e:
        if conn:
            conn.rollback()
        print(f"执行事务失败: {e}")
        print(f"查询列表: {queries_and_params}")
        raise
    finally:
        if cursor:
            cursor.close()
        if conn:
            conn.autocommit = True  # 恢复默认设置
            return_db_connection(conn)

# 为了向后兼容，保留原有的函数名
def get_connection():
    """向后兼容的连接获取函数"""
    return get_db_connection()

# 应用关闭时清理连接池
def cleanup_database():
    """清理数据库资源"""
    close_connection_pool() 