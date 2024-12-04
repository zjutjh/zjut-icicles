package cn.edu.zjut.bean;

public class ItemOrder implements IItemOrder
{
    private IItem item;
    private int numItems;

    public void incrementNumItems()
    {
        setNumItems(getNumItems() + 1);
    }

    public void cancelOrder()
    {
        setNumItems(0);
    }

    public double getTotalCost()
    {
        return (getNumItems() * item.getCost());
    }

    public IItem getItem()
    {
        return item;
    }

    public void setItem(IItem item)
    {
        this.item = item;
    }

    public int getNumItems()
    {
        return numItems;
    }

    public void setNumItems(int numItems)
    {
        this.numItems = numItems;
    }
}