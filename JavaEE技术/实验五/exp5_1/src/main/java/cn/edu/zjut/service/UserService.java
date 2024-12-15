package cn.edu.zjut.service;

import java.util.List;

import cn.edu.zjut.util.HibernateUtil;
import org.hibernate.SessionFactory;
import org.hibernate.Session;
import org.hibernate.cfg.Configuration;
import cn.edu.zjut.po.Customer;
import cn.edu.zjut.dao.CustomerDAO;

public class UserService
{
    //    public Session getSession()
//    {
//        SessionFactory sf = new Configuration().configure().buildSessionFactory();
//        return sf.openSession();
//    }
    public Session getSession()
    {
        return HibernateUtil.getSession();
    }

    public boolean login(Customer loginUser)
    {
        String account = loginUser.getAccount();
        String password = loginUser.getPassword();
        String hql = "from Customer as user where account='" + account + "' and password='" + password + "'";
        Session session = this.getSession();
        CustomerDAO dao = new CustomerDAO();
        dao.setSession(session);
        List list = dao.findByHql(hql);
//        session.close();
        HibernateUtil.closeSession();
        if (list.isEmpty())
            return false;
        else
            return true;
    }
}
