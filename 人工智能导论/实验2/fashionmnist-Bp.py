import numpy as np
import pandas as pd
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
import matplotlib.pyplot as plt
import seaborn as sns
import copy
import time
import torch
import torch.nn as nn
import torch.utils.data as Data
from torchvision import transforms
from torchvision.datasets import FashionMNIST
from torch.nn import functional as F

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
print(torch.__version__)
print(device)
# Hyper Parameters
EPOCH = 64              # train the training data n times, to save time, we just train 1 epoch
BATCH_SIZE = 128
LR = 0.1              # learning rate
Train_rate = 0.8  # 训练集batchsize百分比

# 使用FashionMNIST数据，准备训练数据集
train_data = FashionMNIST(root="./FashionMNIST", train=True,
                          transform=transforms.ToTensor(), download=True)
# 定义一个数据加载器
train_loader = Data.DataLoader(
    dataset=train_data, batch_size=BATCH_SIZE, shuffle=False, num_workers=0)
print(train_data.train_data.size())                 # (60000, 28, 28)
print(train_data.train_labels.size())               # (60000)
# 计算train_loader有多少个batch
print("train_loader的batch数量为：", len(train_loader))
# 获取一个batch的数据
for step, (b_x, b_y) in enumerate(train_loader):
    if step > 0:
        break
# 可视化一个batch的图像
print("b_x:", b_x.shape)  # torch.Size([64, 1, 28, 28])
batch_x = b_x.squeeze().numpy()
batch_y = b_y.numpy()
# batch_x: (64, 28, 28) batch_y: (64,)
print("batch_x:", batch_x.shape, "batch_y:", batch_y.shape)
class_label = train_data.classes

plt.figure(figsize=(12, 5))
for ii in np.arange(len(batch_y)):
    plt.subplot(4, 16, ii % 64 + 1)  # 4行16列
    plt.imshow(batch_x[ii, :, :], cmap=plt.cm.gray)  # batch_x是三维的，顾要加两个分号
    plt.title(class_label[batch_y[ii]], size=9)
    plt.axis("off")
    plt.subplots_adjust(wspace=0.5)  # subplot的间距
plt.show()

# 对测试集进行处理
test_data = FashionMNIST(root="./FashionMNIST", train=False, download=False)
# 为数据添加一个通道维度，并且取值范围缩放到0-1之间
test_data_x = test_data.data.type(torch.FloatTensor) / 255.0
test_data_x = torch.unsqueeze(test_data_x, dim=1)
test_data_y = test_data.targets  # 测试集的标签
# test_data_x.shape torch.Size([10000, 1, 28, 28])
print("test_data_x.shape", test_data_x.shape)
# test_data_y.shape torch.Size([10000])
print("test_data_y.shape", test_data_y.shape)

# 构建BP神经网络网络


class MLP(nn.Module):
    def __init__(self):
        super(MLP, self).__init__()
        # self.model = nn.Sequential(
        #     nn.Linear(28*28, 128),
        #     nn.ReLU(),
        #     nn.Dropout(0.25),
        #     nn.Linear(128, 256),
        #     nn.ReLU(),
        #     nn.Dropout(0.25),
        #     nn.Linear(256, 64),
        #     nn.ReLU(),
        #     nn.Dropout(0.25),
        #     nn.Linear(64, 32),
        #     nn.ReLU(),
        #     nn.Dropout(0.25),
        #     nn.Linear(32, 10),
        #     nn.Softmax(dim=1)
        # )
        self.model = nn.Sequential(
            nn.Linear(28*28, 128),
            nn.ReLU(),
            # nn.Linear(128, 64),
            # nn.ReLU(),
            nn.Linear(128, 10),
            nn.Softmax(dim=1)
        )

    def forward(self, x):
        x = self.model(x)
        return x


# 输出网络结构
mynet = MLP()  # MLP()
mynet = mynet.to(device)  # 网络模型搬移至指定的设备上
print('--------------查看网络结构及参数量-----------')
print(mynet)
total_params = sum(p.numel() for p in mynet.parameters())
print('网络参数量：', total_params)

print("training on ", device)  # 查看当前训练所用的设备
# 定义网络的训练过程函数


def train_model(model, traindataloader, train_rate, criterion, optimizer, num_epochs):
    # train_rate：训练集batchsize百分比
    # 计算训练使用的batch数量
    batch_num = len(traindataloader)
    train_batch_num = round(batch_num * train_rate)  # 前train_rate的batch进行训练
    print("总批数：", batch_num, "， 训练批数：", train_batch_num, "， 训练批量大小：", BATCH_SIZE)
    # 复制模型的参数
    best_model_wts = copy.deepcopy(model.state_dict())
    best_acc = 0.0
    train_loss_all = []
    train_acc_all = []
    val_loss_all = []
    val_acc_all = []
    since = time.time()
    for epoch in range(num_epochs):
        print('Epoch {}/{}'.format(epoch, num_epochs-1))
        print('-' * 10)
        # 每个epoch有两个训练阶段
        train_loss = 0.0
        train_corrects = 0
        train_num = 0
        val_loss = 0.0
        val_corrects = 0
        val_num = 0
        for step, (b_x, b_y) in enumerate(traindataloader):
            b_x, b_y = b_x.to(device), b_y.to(device)  # 将图像、标签搬移至指定设备上
            b_x = b_x.view(-1, 28 * 28)  # BP输入
            # b_x = b_x.to(device)  # 将图像搬移至指定设备上
            if step < train_batch_num:  # 前train_rate的batch进行训练
                model.train()  # 设置模型为训练模式
                output = model(b_x)
                pre_lab = torch.argmax(output, 1)
                loss = criterion(output, b_y)  # loss是一个batch的loss，每个样本的loss均值，
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                # 此处的b_x.size(0)=batch_size。此处相当于一个batch的loss
                train_loss += loss.item() * b_x.size(0)
                train_corrects += torch.sum(pre_lab == b_y.data)
                train_num += b_x.size(0)
            if step < batch_num and step >= train_batch_num:
                model.eval()  # 设置模型为评估模式
                output = model(b_x)
                pre_lab = torch.argmax(output, 1)
                loss = criterion(output, b_y)
                val_loss += loss.item() * b_x.size(0)
                val_corrects += torch.sum(pre_lab == b_y.data)
                val_num += b_x.size(0)
        # 计算一个epoch在训练集和验证集上的损失和精度
        train_loss_all.append(train_loss / train_num)
        train_acc_all.append(train_corrects.double().item() / train_num)
        val_loss_all.append(val_loss / val_num)
        val_acc_all.append(val_corrects.double().item() / val_num)
        print('{} Train Loss: {:.4f} Train Acc: {:.4f}'.format(
            epoch, train_loss_all[-1], train_acc_all[-1]))  # 此处-1没搞明白
        print('{} Val Loss: {:.4f} Val Acc: {:.4f}'.format(
            epoch, val_loss_all[-1], val_acc_all[-1]))
        # 拷贝模型最高精度下的参数
        if val_acc_all[-1] > best_acc:
            best_acc = val_acc_all[-1]
            best_model_wts = copy.deepcopy(model.state_dict())
        time_use = time.time() - since
        print("Train and val complete in {:.0f}m {:.0f}s".format(
            time_use // 60, time_use % 60))
    # 使用最好模型的参数
    model.load_state_dict(best_model_wts)
    # 组成数据表格train_process输出
    train_process = pd.DataFrame(data={"epoch": range(num_epochs),
                                       "train_loss_all": train_loss_all,
                                       "val_loss_all": val_loss_all,
                                       "train_acc_all": train_acc_all,
                                       "val_acc_all": val_acc_all})
    return model, train_process


# 对模型进行训练
# optimizer = torch.optim.Adam(mynet.parameters(), lr=0.0003)
optimizer = torch.optim.SGD(mynet.parameters(),lr=0.0005)
criterion = nn.CrossEntropyLoss().to(device)
mynet, train_process = train_model(
    mynet, train_loader, Train_rate, criterion, optimizer, num_epochs=EPOCH)

# 可视化模型训练过程中
plt.figure(figsize=(12, 4))
plt.subplot(1, 2, 1)
plt.plot(train_process.epoch, train_process.train_loss_all,
         "ro-", label="Train loss")
plt.plot(train_process.epoch, train_process.val_loss_all,
         "bs-", label="Val loss")
plt.legend()
plt.xlabel("epoch")
plt.ylabel("Loss")
plt.subplot(1, 2, 2)
plt.plot(train_process.epoch, train_process.train_acc_all,
         "ro-", label="Train acc")
plt.plot(train_process.epoch, train_process.val_acc_all, "bs-", label="Val acc")
plt.xlabel("epoch")
plt.ylabel("acc")
plt.legend()
plt.show()

# 对测试集进行预测，并可视化预测结果。计算模型的泛化能力
mynet.eval()
# output = mynet(test_data_x.to(device))
output = mynet(test_data_x.to(device).view(-1, 28*28))  # BP输入
# output = mynet(test_data_x.to(device).to(device))
pre_lab = torch.argmax(output, 1).cpu()  # 测试集的预测结果
acc = accuracy_score(test_data_y, pre_lab)  # 计算测试集的正确率
print('--------------查看测试结果-----------')
print("Total: ", acc)
classes = class_label
class_correct = list(0. for i in range(10))
class_total = list(0. for i in range(10))
c = (pre_lab == test_data_y)
for i in range(len(test_data_y)):
    class_correct[test_data_y[i]] += c[i].item()  # 将True/False化为1/0
    class_total[test_data_y[i]] += 1
for i in range(10):
    print("{}:{}%".format(classes[i], 100 * class_correct[i] / class_total[i]))


# 计算混淆矩阵并可视化
conf_mat = confusion_matrix(test_data_y, pre_lab)
# print( conf_mat )#打印混淆矩阵
df_cm = pd.DataFrame(conf_mat, index=class_label, columns=class_label)
heatmap = sns.heatmap(df_cm, annot=True, fmt="d", cmap="YlGnBu")
heatmap.yaxis.set_ticklabels(
    heatmap.yaxis.get_ticklabels(), rotation=0, ha='right')
heatmap.xaxis.set_ticklabels(
    heatmap.xaxis.get_ticklabels(), rotation=45, ha='right')
plt.ylabel('True label')
plt.xlabel('Predicted label')
plt.show()
