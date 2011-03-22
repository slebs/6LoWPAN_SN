<%-- 
    Document   : create
    Created on : Dec 16, 2010, 4:26:29 PM
    Author     : simon
--%>
<%@page import="jdbc.DBManager"%>
<%
            int error = 0;
            int a = 0;
            String benutzerName = (String) session.getAttribute("benutzer");
            System.out.println(benutzerName);
            String newUser = request.getParameter("user");
            String newPassword = request.getParameter("password");

            if (benutzerName != null && DBManager.getInstance().isAdmin(benutzerName) && newUser != null && newPassword != null) {

                String admin = request.getParameter("cb");

                if (admin == null) {
                    a = 0;
                } else {
                    a = 1;
                }
                if (DBManager.getInstance().createNewUser(newUser, newPassword, a)) {
                    System.out.println("setzte attribute");
                    session.setAttribute("justCreated", "ja");
                    response.sendRedirect("login.jsp");
                } else {
                    error = 3;
                    //session.setAttribute("error", "3");
                    //response.sendRedirect("login.jsp");
                }

            }
%>
<%@page contentType="text/html" pageEncoding="UTF-8"%>
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//DE"
    "http://www.w3.org/TR/html4/loose.dtd">
<html>
    <head>
        <title>Create new User</title>
        <meta http-equiv="content-type"
              content="text/html; utf-8">
        <link rel="stylesheet" type="text/css"
              href="html.css" media="all">
    </head>
    <body>
        <div id="wrappertop"></div>
        <div id="wrapper">
            <h1>Create new User</h1>
            <form name="logform" class="form1" action="create.jsp" method="POST">
                <table>
                    <tbody>
                        <tr><td>Name:</td> <td> <input type="text" name="user" value=""/></td></tr>
                        <tr><td>Password:</td> <td> <input type="password" name="password" value=""/></td></tr>
                        <tr><td>Admin:</td> <td><input type="checkbox" name="cb" value="1"></td></tr>
                    </tbody>
                </table>
                <input type="submit" value="Create User" onClick='logform.password.value = Sha1.hash(logform.password)'/>
                <input type="button" name="login" value="Goto Login..."
                       onclick=top.location.href='login.jsp'>
                <% if (error == 3) {%>
                <p>Fehler beim Anlegen des Users</p>
                <%}%>
            </form>
        </div>
            <div id="wrapperbottom"></div>
    </body>
</html>
