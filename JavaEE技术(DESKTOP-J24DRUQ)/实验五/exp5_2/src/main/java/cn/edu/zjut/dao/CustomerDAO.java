package cn.edu.zjut.dao;

import java.util.List;

import cn.edu.zjut.po.Customer;
import org.hibernate.Query;
import org.hibernate.Session;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class CustomerDAO
{
    private Log log = LogFactory.getLog(CustomerDAO.class);
    private Session session;

    public void setSession(Session session)
    {
        this.session = session;
    }

    public List findByHql(String hql)
    {
        log.debug("finding LoginUser instance by hql");
        try
        {
            String queryString = hql;
            Query queryObject = session.createQuery(queryString);
            System.out.println("customer is found");
            return queryObject.list();
        }
        catch (RuntimeException re)
        {
            log.error("find by hql failed", re);
            System.out.println("customer is not found " + re);
            throw re;
        }
        finally
        {
        }
    }

    public void save(Customer customer)
    {
        log.debug("saving customer instance");
        try
        {
            session.save(customer);
            log.debug("save successful");
        }
        catch (RuntimeException re)
        {
            log.error("save failed", re);
            throw re;
        }
        finally
        {
        }
    }
}