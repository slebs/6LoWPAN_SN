<%-- 
    Document   : login.jsp
    Created on : Dec 16, 2010, 4:23:13 PM
    Author     : simon
--%>
<%@page import="serial.SerialComm"%>
<%
            String error = (String) session.getAttribute("error");

            if (error != null) {
                session.removeAttribute("error");
            }
            
%>

<%@page contentType="text/html" pageEncoding="UTF-8"%>
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//DE"
    "http://www.w3.org/TR/html4/loose.dtd">
<html>
    <head>
        <title>Login</title>
        <meta http-equiv="content-type"
              content="text/html; utf-8">
        <link rel="stylesheet" type="text/css"
              href="html.css" media="all">
    </head>
</head>
<body>
    <div id="wrappertop"></div>
    <div id="wrapper">
        <h1>Login</h1>
        <form name="logform" class="form1" action="check.jsp" method="post">
            <table>
                <tbody>
                    <tr><td>Name:</td><td><input type="text" name="user" value="" /></td></tr>
                    <tr><td>Password:</td><td><input type="password" name="password" value=""/></td></tr>
                </tbody>
            </table>
            <input type="submit" value="Login"/>

        </form>
        <%
                    if (error == "1") {
                        out.print("<p>Benutzername oder Passwort falsch</p>");
                        session.setAttribute("error", null);
                    }
                    if ((String) session.getAttribute("justCreated") != null) {
                        out.print("<p>Nutzer erfolgreich angelegt. Bitte melden Sie sich an</p>");
                        session.setAttribute("justCreated", null);
                    }
                    if (error == "3") {
                        out.print("<p>Benutzer NICHT angelegt</p>");
                        session.setAttribute("error", null);
                    }
        %>

    </div>
    <div id="wrapperbottom"></div>
</body>
</html>