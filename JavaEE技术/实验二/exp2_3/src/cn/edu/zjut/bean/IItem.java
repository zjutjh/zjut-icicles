package cn.edu.zjut.bean;

public interface IItem
{
    String getItemID();

    String getTitle();

    String getDescription();

    double getCost();

    void setItemID(String itemID);

    void setTitle(String title);

    void setDescription(String description);

    void setCost(double cost);
}
