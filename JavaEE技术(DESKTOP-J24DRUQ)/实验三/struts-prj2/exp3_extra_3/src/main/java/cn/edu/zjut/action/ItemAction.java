package cn.edu.zjut.action;

import cn.edu.zjut.service.ItemService;
import com.opensymphony.xwork2.ActionSupport;

import java.util.List;


public class ItemAction extends ActionSupport {
    private List items;

    // 省略其他代码

    // Getter方法
    public List getItems() {
        return items;
    }

    // Setter方法
    public void setItems(List items) {
        this.items = items;
    }

    // 省略其他方法
    public String getAllItems() {
        ItemService itemServ = new ItemService();
        items = itemServ.getAllItems();
        System.out.println("ItemAction executed!");
        return "success";
    }
}

