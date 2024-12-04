import numpy as np                
from matplotlib import colors     
from sklearn import svm            
from sklearn.svm import SVC
from sklearn import model_selection
from sklearn.linear_model import LogisticRegression
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
#1.数据准备
#*************将字符串转为整型，便于数据加载***********************
def iris_type(s):
    it = {b'Iris-setosa':0, b'Iris-versicolor':1, b'Iris-virginica':2}
    return it[s]

#1.1加载数据
data_path='C:\\Users\\wangr\\Desktop\\iris.data'          #数据文件的路径
data = np.loadtxt(data_path,                                #数据文件路径
                  dtype=float,                              #数据类型
                  delimiter=',',                            #数据分隔符
                  converters={4:iris_type})                 #将第5列使用函数iris_type进行转换
                                               #data为二维数组，data.shape=(150, 5)
print('数据集规模：',data.shape)
#1.2数据分割
x, y = np.split(data,                                       #要切分的数组
                (4,),                                       #沿轴切分的位置，第5列开始往后为标签y
                axis=1)                                     #代表纵向分割，按列分割
x = x[:, 0:2]                                               #在X中我们取前两列作为特征，为了后面的可视化。x[:,0:2]代表第一维(行)全取，第二维(列)取0,1
#print("输入特征：\n",x)
x_train,x_test,y_train,y_test=model_selection.train_test_split(x,              #所要划分的样本特征集
                                                               y,              #所要划分的样本结果
                                                               random_state=1234, #随机数种子
                                                               test_size=0.4)  #测试样本占比
# 输出划分后的训练样本和测试样本大小
print('train.x',x_train.shape)
print('train.y',y_train.shape)
print('test.x',x_test.shape)
print('test.y',y_test.shape)

#====绘制iris数据集==================
cm_dark = mpl.colors.ListedColormap(['g', 'b', 'r']) 
plt.scatter(x[:, 0], x[:, 1], c=np.squeeze(y), edgecolor='k', s=50, cmap=cm_dark) # 画散点图：所有样本点的前两个特征及标签
#plt.scatter(x_test[:, 0], x_test[:, 1], c=np.squeeze(y_test), s=50, cmap=cm_dark) # 测试点
x1_min, x1_max = x[:, 0].min(), x[:, 0].max()               #第0列的范围
x2_min, x2_max = x[:, 1].min(), x[:, 1].max()               #第1列的范围
plt.xlabel('sepal length', fontsize=20)
plt.ylabel('sepal width', fontsize=20)
plt.xlim(x1_min, x1_max)
plt.ylim(x2_min, x2_max)
plt.title('iris data', fontsize=30)
plt.grid()
plt.show()

#======================
#**********************SVM分类器构建*************************
def classifier(): #定义分类器函数
    clf = svm.SVC(C=0.5,         #支持向量机的误差项惩罚系数,默认值是1，参数说明https://blog.csdn.net/szlcw1/article/details/52336824
                gamma=1.0,
                kernel='rbf',               ##kernel='linear':线性核； kenrel="rbf":高斯核
                decision_function_shape='ovr') #决策函数,即一个类别与其他类别进行划分

    # clf=LogisticRegression(C=1.0,multi_class='ovr', solver='liblinear',tol=0.0001,penalty='l2')#逻辑回归模型,参数说明 https://blog.csdn.net/weixin_46411214/article/details/110090552
    return clf

# 2.定义模型：
clf = classifier() #调用分类器函数
print('采用的分类器model：',clf)
#***********************训练模型*****************************
def train(clf,x_train,y_train): #定义训练函数
    clf.fit(x_train,         #训练集特征向量
            y_train.ravel()) #训练集目标值
    

# 3.训练模型
train(clf,x_train,y_train) #调用训练函数
#查看权值信息
#print("各类别的特征权重",clf.coef_)

#**************并判断a b是否相等，计算准确率acc的均值*************
def show_accuracy(a, b, tip): #定义显示正确率的函数
    acc = a.ravel() == b.ravel()
    print('%s Accuracy:%.3f' %(tip, np.mean(acc)))

from sklearn import metrics
def print_accuracy(clf,x_train,y_train,x_test,y_test): #定义调用显示训练和测试正确率的函数
    #分别打印训练集和测试集的准确率  score(x_train,y_train):表示输出x_train,y_train在模型上的准确率
    #print('training prediction:%.3f' %(clf.score(x_train, y_train)))
    #print('test data prediction:%.3f' %(clf.score(x_test, y_test)))
    new_predict=clf.predict(x_test)
    y_true=y_test
    print('输出预测的标签',new_predict)
    #acc = metrics.accuracy_score(y_true, new_predict)
    f1_macro = metrics.f1_score(y_true, new_predict, average='macro')
    precision_macro = metrics.precision_score(y_true, new_predict, average='macro')
    recall_macro = metrics.recall_score(y_true, new_predict, average='macro')
    #原始结果与预测结果进行对比   predict()表示对x_train样本进行预测，返回样本类别
    show_accuracy(clf.predict(x_train), y_train, 'training data')
    show_accuracy(clf.predict(x_test), y_test, 'testing data')
    print('测试结果：F1macro {:.4f}'.format(f1_macro),', Pmacromacro {:.4f}'.format(precision_macro),', Rmacro {:.4f}'.format(recall_macro))

    #计算决策函数的值，表示x到各分割平面的距离
    #print('decision_function:\n', clf.decision_function(x_train))

# 4.模型评估
print_accuracy(clf,x_train,y_train,x_test,y_test) #输出训练正确率和测试正确率、F1分数、查全率和查准率

#**************绘制输出结果**************
def draw(clf, x):  #定义分类分界面的画图函数
    iris_feature = 'sepal length', 'sepal width', 'petal lenght', 'petal width'
    # 开始画图
    x1_min, x1_max = x[:, 0].min(), x[:, 0].max()               #第0列的范围
    x2_min, x2_max = x[:, 1].min(), x[:, 1].max()               #第1列的范围
    x1, x2 = np.mgrid[x1_min:x1_max:200j, x2_min:x2_max:200j]   #生成网格采样点
    grid_test = np.stack((x1.flat, x2.flat), axis=1)            #stack():沿着新的轴加入一系列数组
    #print('grid_test:\n', grid_test)

    # 输出样本到决策面的距离
    z = clf.decision_function(grid_test)
    #print('the distance to decision plane:\n', z)
    
    grid_hat = clf.predict(grid_test)                           # 预测分类值 得到【0,0.。。。2,2,2】
    #print('grid_hat:\n', grid_hat)
    grid_hat = grid_hat.reshape(x1.shape)                       # reshape grid_hat和x1形状一致
                                                                #若3*3矩阵e，则e.shape()为3*3,表示3行3列   
 
    cm_light = mpl.colors.ListedColormap(['#A0FFA0', '#FFA0A0', '#A0A0FF'])
    cm_dark = mpl.colors.ListedColormap(['g', 'b', 'r'])
    
    plt.pcolormesh(x1, x2, grid_hat, cmap=cm_light)## 区域图 # pcolormesh(x,y,z,cmap)这里参数代入
                                                # x1，x2，grid_hat，cmap=cm_light绘制的是背景。
    plt.scatter(x[:, 0], x[:, 1], c=np.squeeze(y), edgecolor='k', s=50, cmap=cm_dark) # 样本点    
    plt.scatter(x_test[:, 0], x_test[:, 1], c=np.squeeze(y_test), s=50, cmap=cm_dark) # 测试点
    
    plt.xlabel(iris_feature[0], fontsize=20)
    plt.ylabel(iris_feature[1], fontsize=20)
    plt.xlim(x1_min, x1_max)
    plt.ylim(x2_min, x2_max)
    plt.title('SVM in iris data classification', fontsize=30)
    plt.grid()
    plt.show()
    
# 5.模型使用
draw(clf,x) #输出分类分界面的图

#补充：网格搜索调参：模型在所有组超参数上实验，选取交叉验证误差最小的参数。https://www.zhihu.com/tardis/bd/art/473048682?source_id=1001,https://zhuanlan.zhihu.com/p/583484008
from sklearn.model_selection import GridSearchCV #用交叉验证从超参数候选网格中搜索出最佳超参数
#params = {"tol": [1e-4, 1e-3, 1e-2,1e-1],"C": [0.01,0.1,0.5,0.8,1.0,5.0],'penalty':['l1','l2']} #https://www.freesion.com/article/6609953059/#google_vignette
params = {"C":[0.1,0.5,0.8, 1.0,5.0,10.0]}
clf = svm.SVC( #C=0.5, # 误差项惩罚系数,默认值是1 https://blog.csdn.net/weixin_41990278/article/details/93137009
              #gamma=0.1,
              kernel='linear',      #kernel='linear':线性核； kenrel="rbf":高斯核
              decision_function_shape='ovr')  # 决策函数,即一个类别与其他类别进行划分
# clf=LogisticRegression(multi_class='ovr', solver='liblinear')#逻辑回归模型,参数说明 https://blog.csdn.net/weixin_46411214/article/details/110090552
grid=GridSearchCV(clf,params,scoring="f1_macro",cv=5)
grid_result=grid.fit(x_train,y_train)#用训练数据进行网格搜索出模型的最佳超参数
print("最佳结果",grid_result.best_score_)
print("最佳参数",grid_result.best_params_)
print("最佳估计器",grid_result.best_estimator_)
print_accuracy(grid,x_train,y_train,x_test,y_test) #输出训练正确率和测试正确率
print("交叉验证结果:\n",grid_result.cv_results_)