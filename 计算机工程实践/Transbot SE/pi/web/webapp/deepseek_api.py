import requests
import json
import time
import sys
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('deepseek_api')

# 配置信息 - 使用正确的 V2 模型名称
DEEPSEEK_API_KEY = "sk-9c910d72ad7c42d5a9df2e93131950b6"
API_ENDPOINT = "https://api.deepseek.com/chat/completions"
MODEL_NAME = "deepseek-chat"  # 正确的模型名称是 deepseek-chat（对应 V2）

def check_api_key():
    """检查API密钥是否有效"""
    headers = {"Authorization": f"Bearer {DEEPSEEK_API_KEY}"}
    try:
        logger.info("正在验证DeepSeek API密钥...")
        response = requests.get(
            "https://api.deepseek.com/v1/models",
            headers=headers,
            timeout=15
        )
        
        if response.status_code == 200:
            logger.info("✅ API密钥验证成功！")
            models = [m['id'] for m in response.json().get('data', [])]
            logger.info(f"可用模型: {models}")
            print(f"可用模型: {models}")
            return True
        else:
            logger.error(f"❌ 验证失败 (状态码: {response.status_code}): {response.text}")
            print(f"❌ 验证失败 (状态码: {response.status_code}): {response.text}")
            return False
            
    except requests.exceptions.Timeout:
        logger.error("❌ API密钥验证超时")
        print("❌ API密钥验证超时")
        return False
    except Exception as e:
        logger.error(f"❌ 验证过程中出错: {str(e)}")
        print(f"❌ 验证过程中出错: {str(e)}")
        return False

def ask_deepseek(prompt: str) -> str:
    """使用DeepSeek-V2模型获取回答"""
    headers = {
        "Authorization": f"Bearer {DEEPSEEK_API_KEY}",
        "Content-Type": "application/json"
    }
    
    # 正确的消息格式
    payload = {
        "model": MODEL_NAME,
        "messages": [{"role": "user", "content": prompt}],
        "temperature": 0.7,
        "max_tokens": 2000,
        "top_p": 0.9,
        "frequency_penalty": 0,
        "presence_penalty": 0,
        "stop": None
    }
    
    try:
        logger.info(f"正在发送请求到DeepSeek API，提示内容: {prompt[:50]}...")
        
        # 增加超时时间
        response = requests.post(
            API_ENDPOINT,
            json=payload,
            headers=headers,
            timeout=60  # 增加超时时间到60秒
        )
        
        # 调试信息
        logger.info(f"请求状态码: {response.status_code}")
        print(f"请求状态码: {response.status_code}")
        
        if response.status_code != 200:
            error_msg = f"响应内容: {response.text}"
            logger.error(error_msg)
            print(error_msg)
            return f"API调用失败 (状态码: {response.status_code}): {response.text}"
        
        # 解析响应
        response_data = response.json()
        content = response_data["choices"][0]["message"]["content"]
        logger.info(f"成功获取到回答，长度: {len(content)}")
        return content
            
    except requests.exceptions.Timeout:
        error_msg = "请求超时，请稍后再试"
        logger.error(error_msg)
        return error_msg
    except requests.exceptions.ConnectionError:
        error_msg = "连接错误，请检查网络连接"
        logger.error(error_msg)
        return error_msg
    except requests.exceptions.RequestException as e:
        error_msg = f"网络错误: {str(e)}"
        logger.error(error_msg)
        return error_msg
    except (KeyError, IndexError, json.JSONDecodeError) as e:
        error_msg = f"解析响应失败: {str(e)}"
        logger.error(error_msg)
        return error_msg
    except Exception as e:
        error_msg = f"处理请求时出错: {str(e)}"
        logger.error(error_msg)
        return error_msg

def interactive_chat():
    """交互式对话界面"""
    print("\n" + "="*50)
    print(f"DeepSeek-V2 智能助手 (模型: {MODEL_NAME})")
    print("输入 '退出' 或 'exit' 结束程序")
    print("="*50)
    
    while True:
        try:
            user_input = input("\n您: ").strip()
            
            # 检查退出命令
            if user_input.lower() in ["退出", "exit", "quit"]:
                print("\n感谢使用，再见！")
                break
                
            # 处理空输入
            if not user_input:
                print("提示: 请输入有效问题")
                continue
                
            # 显示处理中状态
            print("\nAI思考中...", end="", flush=True)
            
            # 发送请求
            start_time = time.time()
            response = ask_deepseek(user_input)
            processing_time = time.time() - start_time
            
            # 清除"思考中"提示
            print("\r" + " "*15 + "\r", end="")
            
            # 打印响应
            print(f"AI助手: {response}")
            print(f"⏱️ 处理时间: {processing_time:.2f}秒")
            
        except KeyboardInterrupt:
            print("\n\n检测到中断，程序结束。")
            break
        except Exception as e:
            print(f"\n发生错误: {str(e)}")

if __name__ == "__main__":
    # 检查API密钥有效性
    if not check_api_key():
        print("\n程序退出，请解决API密钥问题后再试")
        sys.exit(1)
    
    # 启动交互对话
    interactive_chat()