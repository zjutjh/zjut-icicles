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

    private Session getSession()
    {
        return HibernateUtil.getSession();
    }
}
