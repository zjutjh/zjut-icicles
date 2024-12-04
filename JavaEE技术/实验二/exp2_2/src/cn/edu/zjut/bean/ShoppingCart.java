package cn.edu.zjut.bean;

import java.util.*;

public class ShoppingCart implements IShoppingCart
{
    private List itemsOrdered;

    public List getItemsOrdered()
    {
        return (itemsOrdered);
    }

    public void setItemsOrdered(List itemsOrdered)
    {
        this.itemsOrdered = itemsOrdered;
    }
}