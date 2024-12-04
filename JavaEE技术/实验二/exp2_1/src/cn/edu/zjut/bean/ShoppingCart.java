package cn.edu.zjut.bean;

import java.util.*;

public class ShoppingCart implements IShoppingCart
{
    private Properties itemsOrdered;

    public Properties getItemsOrdered()
    {
        return (itemsOrdered);
    }

    public void setItemsOrdered(Properties itemsOrdered)
    {
        this.itemsOrdered = itemsOrdered;
    }
}