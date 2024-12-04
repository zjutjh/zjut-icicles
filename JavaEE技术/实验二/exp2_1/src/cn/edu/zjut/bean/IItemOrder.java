package cn.edu.zjut.bean;

public interface IItemOrder
{
    void incrementNumItems();

    void cancelOrder();

    double getTotalCost();

    IItem getItem();

    void setItem(IItem item);

    int getNumItems();

    void setNumItems(int numItems);
}

