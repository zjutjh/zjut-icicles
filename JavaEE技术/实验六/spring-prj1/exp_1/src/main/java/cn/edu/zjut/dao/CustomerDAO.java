package cn.edu.zjut.dao;

import org.hibernate.Session;
import cn.edu.zjut.po.Customer;
import org.hibernate.SessionFactory;
import org.hibernate.Transaction;
import org.hibernate.cfg.Configuration;

public class CustomerDAO implements ICustomerDAO
{
    private Session session;
    private SessionFactory sessionFactory;

    public CustomerDAO()
    {
        // 初始化 Hibernate 配置
        Configuration configuration = new Configuration().configure("hibernate.cfg.xml");

        // 创建 SessionFactory
        sessionFactory = configuration.buildSessionFactory();
    }

    public Session getSession()
    {
        SessionFactory sf = new Configuration().configure().buildSessionFactory();
        return sf.openSession();
    }

    public void setSession(Session session)
    {
        this.session = session;
    }

    public void save(Customer transientInstance)
    {
        Session session = getSession();
        session.save(transientInstance);
    }

    @Override
    public void insert(Customer customer)
    {
        Session session = sessionFactory.openSession();
        Transaction transaction = null;

        try
        {
            transaction = session.beginTransaction();

            // 将 customer 对象保存到数据库中
            session.save(customer);

            transaction.commit();
        }
        catch (Exception e)
        {
            if (transaction != null)
            {
                transaction.rollback();
            }
            e.printStackTrace();
        }
        finally
        {
            session.close();
        }
    }
}