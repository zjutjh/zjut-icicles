#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# arguments define
import argparse
import time
# load torch
import torchvision

# other utilities
# import matplotlib.pyplot as plt
from sklearn import svm
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import confusion_matrix

#%% Load the training data
def MNIST_DATASET_TRAIN(downloads, train_amount):
    # Load dataset
    training_data = torchvision.datasets.FashionMNIST(
              root = './FashionMNIST',
              train = True,
              transform = torchvision.transforms.ToTensor(),
              download = downloads
              )
    class_label = training_data.classes
    #Convert Training data to numpy
    train_data = training_data.train_data.numpy()[:train_amount]
    train_label = training_data.train_labels.numpy()[:train_amount]
    # Print training data size
    print('Training data size: ',train_data.shape)
    print('Training data label size:',train_label.shape)   
    # plt.imshow(train_data[0])
    # plt.show()
    train_data = train_data/255.0
    return train_data, train_label,class_label

#%% Load the test data
def MNIST_DATASET_TEST(downloads, test_amount):
    # Load dataset
    testing_data = torchvision.datasets.FashionMNIST(
              root = './FashionMNIST',
              train = False,
              transform = torchvision.transforms.ToTensor(),
              download = downloads
              )
    # Convert Testing data to numpy
    test_data = testing_data.test_data.numpy()[:test_amount]
    test_label = testing_data.test_labels.numpy()[:test_amount]
    # Print training data size
    print('test data size: ',test_data.shape)
    print('test data label size:',test_label.shape)   
    # plt.imshow(test_data[0])
    # plt.show()
    test_data = test_data/255.0
    return test_data, test_label

#%% Main function for MNIST dataset    
if __name__=='__main__':
    # Training Arguments Settings
    parser = argparse.ArgumentParser(description='Saak')
    parser.add_argument('--download_MNIST', default=True, metavar='DL',
                        help='Download MNIST (default: True)')
    parser.add_argument('--train_amount', type=int, default=60000,
                        help='Amount of training samples')
    parser.add_argument('--test_amount', type=int, default=10000,
                        help='Amount of testing samples')
    args = parser.parse_args()
    # Print Arguments
    print('\n----------Argument Values-----------')
    for name, value in vars(args).items():
        print('%s: %s' % (str(name), str(value)))
    print('------------------------------------\n')

    # Load Training Data & Testing Data
    train_data, train_label,class_label  = MNIST_DATASET_TRAIN(args.download_MNIST, args.train_amount)
    test_data, test_label = MNIST_DATASET_TEST(args.download_MNIST, args.test_amount)

    training_features = train_data.reshape(args.train_amount,-1)
    test_features = test_data.reshape(args.test_amount,-1)

    # Training SVM
    print('------Training and testing SVM------')
    since = time.time()
    clf = svm.SVC(C=5.0,gamma=0.05,kernel='rbf',max_iter=100)

    clf.fit(training_features, train_label)
    time_use = time.time() - since
    print("Train complete in {:.0f}m {:.0f}s".format(time_use // 60, time_use % 60))

    #Test on Training data
    train_result = clf.predict(training_features)
    precision = sum(train_result == train_label)/train_label.shape[0]
    print('Training precision: ', precision)

    print('--------------查看测试结果-----------')
    # Test on test data
    test_result = clf.predict(test_features)#测试集的预测结果
    precision = sum(test_result == test_label) / test_label.shape[0]#计算测试集的正确率
    print('Test precision: ', precision)
    #计算每一类别的正确率
    classes =class_label
    class_correct = list(0. for i in range(10))
    class_total = list(0. for i in range(10))
    test_data_y=test_label
    c = (test_result == test_data_y)
    for i in range(len(test_data_y)):
        class_correct[test_data_y[i]] += c[i].item()  # 将True/False化为1/0
        class_total[test_data_y[i]] += 1
    for i in range(10):
        print("accuracy of {}:{}%".format(classes[i], 100 * class_correct[i] / class_total[i]))
    
    #计算混淆矩阵并可视化
    import matplotlib.pyplot as plt
    import pandas as pd
    import seaborn as sns
    conf_mat = confusion_matrix(test_label, test_result)
    #print( conf_mat )#打印混淆矩阵
    df_cm = pd.DataFrame(conf_mat, index=class_label, columns=class_label)
    heatmap = sns.heatmap(df_cm, annot=True, fmt="d", cmap="YlGnBu")
    heatmap.yaxis.set_ticklabels(heatmap.yaxis.get_ticklabels(), rotation=0, ha='right')
    heatmap.xaxis.set_ticklabels(heatmap.xaxis.get_ticklabels(), rotation=45, ha='right')
    plt.ylabel('True label')
    plt.xlabel('Predicted label')
    plt.show()