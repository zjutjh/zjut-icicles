package cn.edu.zjut.pojo;

public class MapUser
{
    private Integer m_uid;
    private String m_uname;
    private String m_usex;

    public Integer getM_uid()
    {
        return m_uid;
    }

    public void setM_uid(Integer m_uid)
    {
        this.m_uid = m_uid;
    }

    public String getM_uname()
    {
        return m_uname;
    }

    public void setM_uname(String m_uname)
    {
        this.m_uname = m_uname;
    }

    public String getM_usex()
    {
        return m_usex;
    }

    public void setM_usex(String m_usex)
    {
        this.m_usex = m_usex;
    }

    public String toString()
    {
        return "User [uid=" + m_uid + ", uname=" + m_uname + ", usex=" + m_usex + "]";
    }
}
