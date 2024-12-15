package cn.edu.zjut.service;

import cn.edu.zjut.dao.BaseHibernate;
import cn.edu.zjut.dao.ICustomerDAO;
import org.hibernate.Session;
import org.hibernate.Transaction;
import cn.edu.zjut.po.Customer;

public class UserService extends BaseHibernate
{
    private ICustomerDAO customerDAO = null;

    public void setCustomerDAO(ICustomerDAO customerDAO)
    {
        this.customerDAO = customerDAO;
    }

    public void saveUser(Customer c)
    {
        Transaction tran = null;
        Session session = null;
        try
        {
            session = getSession();
            tran = session.beginTransaction();
            customerDAO.setSession(session);
            customerDAO.insert(c);
            tran.commit();

            // 添加操作成功的打印信息
            System.out.println("操作成功！用户已保存。");
        }
        catch (RuntimeException re)
        {
            if (tran != null)
                tran.rollback();
            throw re;
        }
        finally
        {
            session.close();
        }
    }
}
