from munkres import Munkres
import numpy as np
from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn import datasets
import matplotlib.pyplot as plt
from sklearn.metrics import precision_score, recall_score

def cluster_acc(y_true, y_pred):
    y_true = y_true - np.min(y_true)
    l1 = list(set(y_true))
    numclass1 = len(l1)
    l2 = list(set(y_pred))
    numclass2 = len(l2)
    ind = 0
    if numclass1 != numclass2:
        for i in l1:
            if i in l2:
                pass
            else:
                y_pred[ind] = i
                ind += 1
    l2 = list(set(y_pred))
    numclass2 = len(l2)
    if numclass1 != numclass2:
        print('error')
        return
    cost = np.zeros((numclass1, numclass2), dtype=int)
    for i, c1 in enumerate(l1):
        mps = [i1 for i1, e1 in enumerate(y_true) if e1 == c1]
        for j, c2 in enumerate(l2):
            mps_d = [i1 for i1 in mps if y_pred[i1] == c2]
            cost[i][j] = len(mps_d)
    # match two clustering results by Munkres algorithm
    m = Munkres()
    cost = cost.__neg__().tolist()
    indexes = m.compute(cost)
    # get the match results
    new_predict = np.zeros(len(y_pred))
    for i, c in enumerate(l1):
        # correponding label in l2:
        c2 = l2[indexes[i][1]]
        # ai is the index with label==c2 in the pred_label list
        ai = [ind for ind, elm in enumerate(y_pred) if elm == c2]
        new_predict[ai] = c
    acc = metrics.accuracy_score(y_true, new_predict)
    f1_macro = metrics.f1_score(y_true, new_predict, average='macro')
    return acc, f1_macro,new_predict

#标签映射后正确率的计算示例
def iris_type(s):
    it = {b'Iris-setosa':0, b'Iris-versicolor':1, b'Iris-virginica':2}
    return it[s]

iris = datasets.load_iris()

X = iris.data[:, :2]
# print("y", iris.target)

estimator = KMeans(n_clusters=3)

estimator.fit(X)

labels_pred = estimator.labels_

print("聚类标签：\n", labels_pred)
print("轮廓系数：", metrics.silhouette_score(X, labels_pred, metric='euclidean'))

acc, f1 ,new_predict= cluster_acc(iris.target, labels_pred)
precision_macro = precision_score(iris.target, labels_pred, average='macro')
recall_macro = recall_score(iris.target, labels_pred, average='macro')
print("新聚类标签:\n",new_predict)
print('正确率acc： {:.4f}'.format(acc), ', f1分数： {:.4f}'.format(f1))
print("查准率：", precision_macro, "查全率：", recall_macro)

plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.scatter(X[:, 0], X[:, 1], c=iris.target, s=50, cmap='viridis')
plt.xlabel('Petal Length', fontsize=16)
plt.ylabel('Petal Width', fontsize=16)
plt.title('Original Dataset', fontsize=20)

plt.subplot(1, 2, 2)
plt.scatter(X[:, 0], X[:, 1], c=estimator.labels_, s=50, cmap='viridis')
plt.scatter(estimator.cluster_centers_[:, 0], estimator.cluster_centers_[:, 1], c='red', s=200, marker='X')
plt.xlabel('Petal Length', fontsize=16)
plt.ylabel('Petal Width', fontsize=16)
plt.title('K-Means Clustering Result', fontsize=20)

plt.tight_layout()
plt.show()