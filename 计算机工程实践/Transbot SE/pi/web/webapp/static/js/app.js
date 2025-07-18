document.addEventListener('DOMContentLoaded', function() {
    // 初始化变量和元素引用
    const ordersList = document.getElementById('ordersList');
    const takingOrdersList = document.getElementById('takingOrdersList');
    const takenOrdersList = document.getElementById('takenOrdersList');
    const deliveredOrdersList = document.getElementById('deliveredOrdersList');
    const chatMessages = document.getElementById('chatMessages');
    const chatInput = document.getElementById('chatInput');
    const sendMessageBtn = document.getElementById('sendMessageBtn');
    const refreshOrdersBtn = document.getElementById('refreshOrdersBtn');
    const refreshTakingOrdersBtn = document.getElementById('refreshTakingOrdersBtn');
    const refreshTakenOrdersBtn = document.getElementById('refreshTakenOrdersBtn');
    const refreshDeliveredOrdersBtn = document.getElementById('refreshDeliveredOrdersBtn');
    const refreshAllBtn = document.getElementById('refreshAllBtn');
    const takeOrderModal = new bootstrap.Modal(document.getElementById('takeOrderModal'));
    const orderIdInput = document.getElementById('orderIdInput');
    const orderInfoDisplay = document.getElementById('orderInfoDisplay');
    const studentIdInput = document.getElementById('studentIdInput');
    const studentNameInput = document.getElementById('studentNameInput');
    const confirmTakeOrderBtn = document.getElementById('confirmTakeOrderBtn');
    
    // 语音识别和语音合成相关元素
    const voiceInputBtn = document.getElementById('voiceInputBtn');
    const recordingModal = new bootstrap.Modal(document.getElementById('recordingModal'));
    const stopRecordingModalBtn = document.getElementById('stopRecordingModalBtn');
    
    // 二维码扫描相关元素
    const qrCodeInput = document.getElementById('qrCodeInput');
    const scanQrCodeBtn = document.getElementById('scanQrCodeBtn');
    const qrCodeResult = document.getElementById('qrCodeResult');
    

    
    // 音频状态管理
    let isAudioMuted = false;
    let audioToggleBtn = null;
    
    // 创建音频切换按钮
    function createAudioToggleButton() {
        // 为普通AI助手创建音频切换按钮
        const chatInput = document.querySelector('.chat-input .input-group-append');
        if (chatInput) {
            const normalChatAudioBtn = document.createElement('button');
            normalChatAudioBtn.className = 'btn btn-sm btn-outline-secondary ms-2 audio-toggle-btn';
            normalChatAudioBtn.title = '切换音频播放';
            normalChatAudioBtn.addEventListener('click', toggleAudioMute);
            chatInput.appendChild(normalChatAudioBtn);
            audioToggleBtn = normalChatAudioBtn;
        }
        
        updateAudioToggleButton();
        return audioToggleBtn;
    }
    
    // 更新音频切换按钮显示
    function updateAudioToggleButton() {
        // 获取所有音频切换按钮
        const audioButtons = document.querySelectorAll('.audio-toggle-btn');
        
        audioButtons.forEach(button => {
            if (isAudioMuted) {
                button.innerHTML = '<i class="fas fa-volume-mute text-danger"></i>';
                button.title = '当前静音，点击启用音频';
                button.className = 'btn btn-sm btn-outline-danger ms-2 audio-toggle-btn';
            } else {
                button.innerHTML = '<i class="fas fa-volume-up text-success"></i>';
                button.title = '当前音频正常，点击静音';
                button.className = 'btn btn-sm btn-outline-success ms-2 audio-toggle-btn';
            }
        });
    }
    
    // 切换音频静音状态
    function toggleAudioMute() {
        isAudioMuted = !isAudioMuted;
        updateAudioToggleButton();
        
        // 保存到localStorage
        localStorage.setItem('audioMuted', isAudioMuted.toString());
        
        // 显示状态提示
        const statusText = isAudioMuted ? '音频已静音' : '音频已开启';
        showAudioStatusToast(statusText);
    }
    
    // 显示音频状态提示
    function showAudioStatusToast(message) {
        // 创建一个简单的toast提示
        const toast = document.createElement('div');
        toast.className = 'position-fixed bottom-0 end-0 m-3 alert alert-info alert-dismissible fade show audio-status-toast';
        toast.style.zIndex = '9999';
        toast.innerHTML = `
            <i class="fas fa-${isAudioMuted ? 'volume-mute' : 'volume-up'} me-2"></i>
            ${message}
            <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
        `;
        
        document.body.appendChild(toast);
        
        // 3秒后自动消失
        setTimeout(() => {
            if (toast.parentNode) {
                toast.parentNode.removeChild(toast);
            }
        }, 3000);
    }
    
    // 播放音频函数
    function playAudio(audioUrl) {
        if (isAudioMuted || !audioUrl) return;
        
        try {
            // 创建新的音频对象
            const audio = new Audio(audioUrl);
            audio.volume = 0.8; // 设置音量
            
            // 播放音频
            const playPromise = audio.play();
            
            if (playPromise !== undefined) {
                playPromise
                    .then(() => {
                        console.log('音频播放成功');
                    })
                    .catch(error => {
                        console.log('音频播放失败，可能需要用户交互:', error);
                        // 如果自动播放失败，显示提示
                        showAudioPlayFailToast();
                    });
            }
        } catch (error) {
            console.error('音频播放错误:', error);
        }
    }
    
    // 显示音频播放失败提示
    function showAudioPlayFailToast() {
        const toast = document.createElement('div');
        toast.className = 'position-fixed bottom-0 end-0 m-3 alert alert-warning alert-dismissible fade show';
        toast.style.zIndex = '9999';
        toast.innerHTML = `
            <i class="fas fa-exclamation-triangle me-2"></i>
            浏览器限制自动播放，请点击页面任意位置后重试
            <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
        `;
        
        document.body.appendChild(toast);
        
        setTimeout(() => {
            if (toast.parentNode) {
                toast.parentNode.removeChild(toast);
            }
        }, 5000);
    }
    
    // 初始化音频状态
    function initializeAudioSettings() {
        // 从localStorage读取音频状态
        const savedMuteState = localStorage.getItem('audioMuted');
        if (savedMuteState !== null) {
            isAudioMuted = savedMuteState === 'true';
        }
        
        // 创建音频切换按钮
        createAudioToggleButton();
    }
    
    // 加载订单列表
    function loadOrders() {
        ordersList.innerHTML = '<div class="text-center py-5"><div class="spinner-border text-primary" role="status"></div><p class="mt-2">加载中...</p></div>';
        
        fetch('/api/orders')
            .then(response => response.json())
            .then(orders => {
                if (orders.length === 0) {
                    ordersList.innerHTML = '<div class="text-center py-4"><i class="fas fa-inbox fa-3x text-muted mb-3"></i><p>暂无待取订单</p></div>';
                    return;
                }
                
                let html = '';
                orders.forEach(order => {
                    html += `
                        <div class="order-card fade-in">
                            <div class="d-flex justify-content-between align-items-center mb-2">
                                <span class="status-badge status-available">可取</span>
                                <small class="text-muted">${order.timestamp}</small>
                            </div>
                            <h5 class="mb-3">${order.name}</h5>
                            <div class="d-flex justify-content-between align-items-center">
                                <span class="order-id">订单号: ${order.order_id}</span>
                                <button class="btn btn-sm btn-primary take-order-btn" data-id="${order.order_id}" data-name="${order.name}">
                                    <i class="fas fa-hand-paper me-1"></i>取餐
                                </button>
                            </div>
                        </div>
                    `;
                });
                
                ordersList.innerHTML = html;
                
                // 为取餐按钮添加事件监听
                const takeOrderBtns = document.querySelectorAll('.take-order-btn');
                takeOrderBtns.forEach(btn => {
                    btn.addEventListener('click', function() {
                        const orderId = this.getAttribute('data-id');
                        const orderName = this.getAttribute('data-name');
                        orderIdInput.value = orderId;
                        orderInfoDisplay.innerHTML = `<strong>外卖名称:</strong> ${orderName}<br><strong>订单编号:</strong> ${orderId}`;
                        takeOrderModal.show();
                    });
                });
            })
            .catch(error => {
                console.error('加载订单失败:', error);
                ordersList.innerHTML = '<div class="alert alert-danger">加载订单失败，请刷新重试</div>';
            });
    }
    
    // 加载正在取餐订单列表
    function loadTakingOrders() {
        takingOrdersList.innerHTML = '<div class="text-center py-5"><div class="spinner-border text-primary" role="status"></div><p class="mt-2">加载中...</p></div>';
        
        fetch('/api/taking_orders')
            .then(response => response.json())
            .then(orders => {
                if (orders.length === 0) {
                    takingOrdersList.innerHTML = '<div class="text-center py-4"><i class="fas fa-clock fa-3x text-muted mb-3"></i><p>暂无正在取餐订单</p></div>';
                    return;
                }
                
                let html = '';
                orders.forEach(order => {
                    html += `
                        <div class="order-card fade-in">
                            <div class="d-flex justify-content-between align-items-center mb-2">
                                <span class="status-badge status-taking">取餐中</span>
                                <small class="text-muted">${order.taking_start_timestamp}</small>
                            </div>
                            <h5 class="mb-2">${order.name}</h5>
                            <div class="mb-2">
                                <span class="order-id">订单号: ${order.order_id}</span>
                            </div>
                            <div class="text-muted small">
                                <i class="fas fa-user me-1"></i>取餐人: ${order.student_name} (${order.student_id})
                            </div>
                        </div>
                    `;
                });
                
                takingOrdersList.innerHTML = html;
            })
            .catch(error => {
                console.error('加载正在取餐订单失败:', error);
                takingOrdersList.innerHTML = '<div class="alert alert-danger">加载正在取餐订单失败，请刷新重试</div>';
            });
    }

    // 加载已取订单列表
    function loadTakenOrders() {
        takenOrdersList.innerHTML = '<div class="text-center py-5"><div class="spinner-border text-primary" role="status"></div><p class="mt-2">加载中...</p></div>';
        
        fetch('/api/taken_orders')
            .then(response => response.json())
            .then(orders => {
                if (orders.length === 0) {
                    takenOrdersList.innerHTML = '<div class="text-center py-4"><i class="fas fa-truck fa-3x text-muted mb-3"></i><p>暂无配送订单</p></div>';
                    return;
                }
                
                let html = '';
                orders.forEach(order => {
                    html += `
                        <div class="order-card fade-in">
                            <div class="d-flex justify-content-between align-items-center mb-2">
                                <span class="status-badge status-taken">配送中</span>
                                <small class="text-muted">${order.taken_timestamp}</small>
                            </div>
                            <h5 class="mb-2">${order.name}</h5>
                            <div class="mb-2">
                                <span class="order-id">订单号: ${order.order_id}</span>
                            </div>
                            <div class="text-muted small">
                                <i class="fas fa-user me-1"></i>取餐人: ${order.student_name} (${order.student_id})
                            </div>
                        </div>
                    `;
                });
                
                takenOrdersList.innerHTML = html;
            })
            .catch(error => {
                console.error('加载配送列表失败:', error);
                takenOrdersList.innerHTML = '<div class="alert alert-danger">加载配送列表失败，请刷新重试</div>';
            });
    }

    // 加载已送达订单列表
    function loadDeliveredOrders() {
        deliveredOrdersList.innerHTML = '<div class="text-center py-5"><div class="spinner-border text-primary" role="status"></div><p class="mt-2">加载中...</p></div>';
        
        fetch('/api/delivered_orders')
            .then(response => response.json())
            .then(orders => {
                if (orders.length === 0) {
                    deliveredOrdersList.innerHTML = '<div class="text-center py-4"><i class="fas fa-check-circle fa-3x text-muted mb-3"></i><p>暂无已送达订单</p></div>';
                    return;
                }
                
                let html = '';
                orders.forEach(order => {
                    html += `
                        <div class="order-card fade-in">
                            <div class="d-flex justify-content-between align-items-center mb-2">
                                <span class="status-badge status-delivered">已送达</span>
                                <small class="text-muted">${order.delivered_timestamp}</small>
                            </div>
                            <h5 class="mb-2">${order.name}</h5>
                            <div class="mb-2">
                                <span class="order-id">订单号: ${order.order_id}</span>
                            </div>
                            <div class="text-muted small">
                                <i class="fas fa-user me-1"></i>取餐人: ${order.student_name} (${order.student_id})
                                ${order.robot_id ? `<br><i class="fas fa-robot me-1"></i>配送机器人: ${order.robot_id}` : ''}
                            </div>
                        </div>
                    `;
                });
                
                deliveredOrdersList.innerHTML = html;
            })
            .catch(error => {
                console.error('加载已送达订单失败:', error);
                deliveredOrdersList.innerHTML = '<div class="alert alert-danger">加载已送达订单失败，请刷新重试</div>';
            });
    }
    
    // 发送消息到AI
    function sendMessage() {
        const message = chatInput.value.trim();
        if (!message) return;
        
        // 添加用户消息到聊天窗口
        const userMessageElement = document.createElement('div');
        userMessageElement.className = 'message user-message fade-in';
        userMessageElement.textContent = message;
        chatMessages.appendChild(userMessageElement);
        
        // 滚动到底部
        chatMessages.scrollTop = chatMessages.scrollHeight;
        
        // 清空输入框
        chatInput.value = '';
        
        // 显示加载动画
        const loaderElement = document.createElement('div');
        loaderElement.className = 'message ai-message fade-in';
        loaderElement.innerHTML = '<div class="loader"></div>正在思考...';
        chatMessages.appendChild(loaderElement);
        chatMessages.scrollTop = chatMessages.scrollHeight;
        
        // 设置超时处理
        let isTimeout = false;
        const timeoutId = setTimeout(() => {
            isTimeout = true;
            if (chatMessages.contains(loaderElement)) {
                chatMessages.removeChild(loaderElement);
                
                // 显示超时消息
                const timeoutMessageElement = document.createElement('div');
                timeoutMessageElement.className = 'message ai-message fade-in';
                timeoutMessageElement.innerHTML = '<i class="fas fa-clock text-warning"></i> 响应超时，但请求仍在处理中。请稍等片刻，或尝试重新发送消息。';
                chatMessages.appendChild(timeoutMessageElement);
                chatMessages.scrollTop = chatMessages.scrollHeight;
            }
        }, 30000); // 30秒超时
        
        console.log("发送聊天请求:", message); // 添加调试日志
        
        // 发送请求到统一的聊天API
        fetch('/api/chat', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ message })
        })
        .then(response => {
            console.log("收到响应状态:", response.status); // 添加调试日志
            clearTimeout(timeoutId);
            return response.json();
        })
        .then(data => {
            console.log("收到响应数据:", data); // 添加调试日志
            
            // 如果已经超时，不再移除加载动画（已被移除）
            if (!isTimeout && chatMessages.contains(loaderElement)) {
                chatMessages.removeChild(loaderElement);
            }
            
            // 添加AI回复
            const aiMessageElement = document.createElement('div');
            aiMessageElement.className = 'message ai-message fade-in';
            
            if (data.status === 'success') {
                aiMessageElement.textContent = data.response;
                
                chatMessages.appendChild(aiMessageElement);
                
                // 直接播放音频（不显示控件）
                if (data.audio_url) {
                    playAudio(data.audio_url);
                }
            } else {
                aiMessageElement.innerHTML = `<i class="fas fa-exclamation-triangle text-warning"></i> 抱歉，我遇到了一些问题：${data.message || '请稍后再试'}`;
                chatMessages.appendChild(aiMessageElement);
            }
            
            chatMessages.scrollTop = chatMessages.scrollHeight;
        })
        .catch(error => {
            console.error("聊天请求错误:", error); // 添加调试日志
            clearTimeout(timeoutId);
            
            // 如果已经超时，不再移除加载动画（已被移除）
            if (!isTimeout && chatMessages.contains(loaderElement)) {
                chatMessages.removeChild(loaderElement);
            }
            
            // 显示错误消息
            const errorMessageElement = document.createElement('div');
            errorMessageElement.className = 'message ai-message fade-in';
            errorMessageElement.innerHTML = '<i class="fas fa-exclamation-triangle text-danger"></i> 网络错误，请检查连接后重试。如果问题持续存在，可能是服务器负载过高，请稍后再试。';
            chatMessages.appendChild(errorMessageElement);
            chatMessages.scrollTop = chatMessages.scrollHeight;
        });
    }
    
    // 确认取餐
    function takeOrder() {
        const orderId = orderIdInput.value;
        const studentId = studentIdInput.value.trim();
        const studentName = studentNameInput.value.trim();
        
        if (!orderId || !studentId || !studentName) {
            alert('请完整填写信息');
            return;
        }
        
        confirmTakeOrderBtn.disabled = true;
        confirmTakeOrderBtn.innerHTML = '<span class="spinner-border spinner-border-sm" role="status" aria-hidden="true"></span> 处理中...';
        
        fetch('/api/take_order', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                order_id: orderId,
                student_id: studentId,
                student_name: studentName
            })
        })
        .then(response => response.json())
        .then(data => {
            takeOrderModal.hide();
            
            if (data.status === 'success') {
                // 刷新所有列表
                loadOrders();
                loadTakingOrders();
                loadTakenOrders();
                loadDeliveredOrders();
                
                // 显示成功提示
                const successToast = document.createElement('div');
                successToast.className = 'toast align-items-center text-white bg-success border-0 position-fixed bottom-0 end-0 m-3';
                successToast.setAttribute('role', 'alert');
                successToast.setAttribute('aria-live', 'assertive');
                successToast.setAttribute('aria-atomic', 'true');
                successToast.innerHTML = `
                    <div class="d-flex">
                        <div class="toast-body">
                            <i class="fas fa-clock me-2"></i>开始取餐流程，请等待小车配送！
                        </div>
                        <button type="button" class="btn-close btn-close-white me-2 m-auto" data-bs-dismiss="toast"></button>
                    </div>
                `;
                document.body.appendChild(successToast);
                
                const bsToast = new bootstrap.Toast(successToast);
                bsToast.show();
                
                // 清空表单
                document.getElementById('takeOrderForm').reset();
            } else {
                alert('取餐失败：' + data.message);
            }
        })
        .catch(error => {
            console.error('取餐请求失败:', error);
            alert('网络错误，请稍后重试');
        })
        .finally(() => {
            confirmTakeOrderBtn.disabled = false;
            confirmTakeOrderBtn.innerHTML = '确认取餐';
        });
    }
    
    // 处理订单上传
    function processQrCode() {
        const code = qrCodeInput.value.trim();
        if (!code) {
            qrCodeResult.innerHTML = '<div class="qr-error">请输入订单编号</div>';
            return;
        }
        
        // 验证订单号格式（数字）
        if (!/^\d+$/.test(code)) {
            qrCodeResult.innerHTML = '<div class="qr-error">订单编号只能包含数字</div>';
            return;
        }
        
        // 检查是否为支持的订单号
        const supportedOrders = ['15701809', '50924357', '11642704'];
        if (!supportedOrders.includes(code)) {
            qrCodeResult.innerHTML = '<div class="qr-error">不支持的订单编号，请输入：15701809、50924357 或 11642704</div>';
            return;
        }
        
        // 显示加载状态
        qrCodeResult.innerHTML = '<div class="text-center"><div class="spinner-border text-primary" role="status"></div><p class="mt-2">上传中...</p></div>';
        
        // 发送请求到API
        fetch('/api/qrcode', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ code })
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'success') {
                qrCodeResult.innerHTML = `
                    <div class="qr-success">
                        <h5>上传成功</h5>
                        <p>${data.response_text}</p>
                        <div class="mt-2">
                            <button class="btn btn-sm btn-primary" onclick="location.reload()">刷新列表</button>
                        </div>
                    </div>
                `;
                // 清空输入框
                qrCodeInput.value = '';
                // 刷新订单列表
                loadOrders();
            } else {
                qrCodeResult.innerHTML = `<div class="qr-error">上传失败：${data.message}</div>`;
            }
        })
        .catch(error => {
            console.error('订单上传失败:', error);
            qrCodeResult.innerHTML = '<div class="qr-error">网络错误，请稍后重试</div>';
        });
        }

    // 完整的语音识别功能
    function startVoiceRecognition() {
        // 显示录音模态框
        recordingModal.show();
        
        // 发送请求到语音识别API
        fetch('/api/voice_recognition', {
            method: 'POST'
        })
        .then(response => response.json())
        .then(data => {
            recordingModal.hide();
            
            if (data.status === 'success') {
                // 将识别结果填入聊天输入框
                chatInput.value = data.text;
                
                // 显示成功提示
                showVoiceToast('语音识别成功！', 'success');
            } else {
                showVoiceToast('语音识别失败：' + data.message, 'error');
            }
        })
        .catch(error => {
            recordingModal.hide();
            console.error('语音识别失败:', error);
            showVoiceToast('网络错误，请稍后重试', 'error');
        });
    }
    
    // 显示语音操作提示
    function showVoiceToast(message, type = 'info') {
        const toast = document.createElement('div');
        const alertClass = type === 'success' ? 'alert-success' : type === 'error' ? 'alert-danger' : 'alert-info';
        const icon = type === 'success' ? 'fa-check-circle' : type === 'error' ? 'fa-exclamation-triangle' : 'fa-info-circle';
        
        toast.className = `position-fixed top-0 end-0 m-3 alert ${alertClass} alert-dismissible fade show audio-status-toast`;
        toast.style.zIndex = '9999';
        toast.innerHTML = `
            <i class="fas ${icon} me-2"></i>
            ${message}
            <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
        `;
        
        document.body.appendChild(toast);
        
        // 3秒后自动消失
        setTimeout(() => {
            if (toast.parentNode) {
                toast.parentNode.removeChild(toast);
            }
        }, 3000);
    }

    // 语音输入按钮点击事件
    voiceInputBtn.addEventListener('click', function() {
        startVoiceRecognition();
    });
    
    // 模态框中的停止录音按钮点击事件
    stopRecordingModalBtn.addEventListener('click', function() {
        recordingModal.hide();
    });

    
    


    
    // 上传订单按钮点击事件
    scanQrCodeBtn.addEventListener('click', function() {
        processQrCode();
    });
    

    
    // 事件监听器
    sendMessageBtn.addEventListener('click', sendMessage);
    chatInput.addEventListener('keypress', function(e) {
        if (e.key === 'Enter') sendMessage();
    });
    
    refreshOrdersBtn.addEventListener('click', loadOrders);
    refreshTakingOrdersBtn.addEventListener('click', loadTakingOrders);
    refreshTakenOrdersBtn.addEventListener('click', loadTakenOrders);
    refreshDeliveredOrdersBtn.addEventListener('click', loadDeliveredOrders);
    refreshAllBtn.addEventListener('click', function() {
        loadOrders();
        loadTakingOrders();
        loadTakenOrders();
        loadDeliveredOrders();
    });
    
    confirmTakeOrderBtn.addEventListener('click', takeOrder);
    
    // 订单号输入框回车事件
    qrCodeInput.addEventListener('keypress', function(e) {
        if (e.key === 'Enter') processQrCode();
    });
    

    
    // 初始化加载
    loadOrders();
    loadTakingOrders();
    loadTakenOrders();
    loadDeliveredOrders();
    
    // 初始化音频状态
    initializeAudioSettings();
}); 