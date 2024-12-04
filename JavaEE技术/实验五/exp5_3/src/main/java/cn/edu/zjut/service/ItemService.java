package cn.edu.zjut.service;

import java.util.List;
import java.util.ArrayList;

import cn.edu.zjut.dao.ItemDAO;
import org.hibernate.Session;
import cn.edu.zjut.util.HibernateUtil;

public class ItemService
{
    private List items = new ArrayList();

    public List getAllItems()
    {
        Session session = this.getSession();
        ItemDAO dao = new ItemDAO();
        dao.setSession(session);
        List items = dao.findAll();
        HibernateUtil.closeSession();
        return items;
    }

    public List findByHql()
    {
        Session session = this.getSession();
        ItemDAO dao = new ItemDAO();
        dao.setSession(session);
//        String hql = "from cn.edu.zjut.po.Item";
//        String hql = "from Item";
//        String hql = "from Item as item";
//        String hql = "select item.ipk.title from Item as item";
        String hql = "select item.ipk.title, item.cost from Item as item";
        List list = dao.findByHql(hql);
        HibernateUtil.closeSession();
        return list;
    }

    private Session getSession()
    {
        return HibernateUtil.getSession();
    }
}
