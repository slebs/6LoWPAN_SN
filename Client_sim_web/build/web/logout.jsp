<%-- 
    Document   : logout
    Created on : Dec 16, 2010, 5:15:39 PM
    Author     : simon
--%>
<%
            session.setAttribute("benutzer", null);
            session.removeAttribute("benutzer");
            session.invalidate();
            response.sendRedirect("login.jsp");
%>