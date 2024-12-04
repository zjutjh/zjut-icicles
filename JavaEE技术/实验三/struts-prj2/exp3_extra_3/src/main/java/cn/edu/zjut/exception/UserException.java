package cn.edu.zjut.exception;

public class UserException extends Exception
{
    public UserException()
    {
        super();
    }

    public UserException(String msg)
    {
        super(msg);
    }

    public UserException(String msg, Throwable cause)
    {
        super(msg, cause);
    }

    public UserException(Throwable cause)
    {
        super(cause);
    }
}