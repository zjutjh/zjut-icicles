package cn.edu.zjut.po;

public class IDcard
{
    private int id;
    private String code;

    public IDcard()
    {
    }

    public IDcard(int id, String code)
    {
        this.id = id;
        this.code = code;
    }

    public int getId()
    {
        return id;
    }

    public void setId(int id)
    {
        this.id = id;
    }

    public String getCode()
    {
        return code;
    }

    public void setCode(String code)
    {
        this.code = code;
    }

    @Override
    public String toString()
    {
        return "IDcard{" + "id=" + id + ", code='" + code + '\'' + '}';
    }
}