package cn.edu.zjut.dao;

import java.util.List;
import org.hibernate.Query;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.hibernate.Session;

public class ItemDAO {
    private static final Log log = LogFactory.getLog(ItemDAO.class);
    private Session session;

    public void setSession(Session session) {
        this.session = session;
    }

    public List findAll() {
        log.debug("finding all Item instances");
        try {
            String queryString = "from Item";
            Query queryObject = session.createQuery(queryString);
            return queryObject.list();
        } catch (RuntimeException re) {
            log.error("find all failed", re);
            throw re;
        }
    }
}
