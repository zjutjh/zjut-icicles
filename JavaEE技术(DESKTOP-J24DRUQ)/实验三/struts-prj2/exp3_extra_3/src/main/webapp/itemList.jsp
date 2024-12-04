<%@ page language="java" contentType="text/html; charset=GB18030"
         pageEncoding="GB18030"
         import="com.opensymphony.xwork2.util.ValueStack,
                 java.util.List,
                 java.util.Iterator,
                 cn.edu.zjut.bean.Item" %>
<%@ taglib prefix="s" uri="/struts-tags" %>
<body>
<s:property value="#session.user"/>，您好！
<center>商品列表</center>
<table border=1>
    <tr>
        <th>编号</th><th>书名</th><th>说明</th><th>单价</th>
    </tr>
    <s:iterator value="items" >
        <tr>
            <td><s:property value="itemID"/></td>
            <td><s:property value="name"/></td>
            <td><s:property value="description"/></td>
            <td><s:property value="cost"/></td>
        </tr>
    </s:iterator>
    价格低于 20 元的商品有：<br>
    <s:iterator value="items.{?#this.cost<20}" >
        <li><s:property value="title"/>：
            <s:property value="cost" />元</li>
    </s:iterator>
    <p>
        名称为《JAVAEE 技术实验指导教程》的商品的价格为：
        <s:property value="items.{?#this.title=='JAVAEE 技术实验指导教程'}.{cost}[0]"/>元
    <s:url value="items.{title}[0]"/><br>
    <s:url value="%{items.{title}[0]}"/>
</table>
</body>