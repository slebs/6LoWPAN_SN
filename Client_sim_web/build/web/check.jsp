<%@page import="jdbc.DBManager"%>
<%
    String benutzerName = request.getParameter("user");
    String passWord = request.getParameter("password");

    if (benutzerName != null && passWord != null && DBManager.getInstance().validiereUser(benutzerName, passWord)) {
        session.setAttribute("benutzer", benutzerName);
        response.sendRedirect("content.jsp");
        
        System.out.println("User validierung erfolgreich");
    } else {
        session.setAttribute("error", "1");
        response.sendRedirect("login.jsp");
        System.out.println("!!User validierung fehlgeschlagen");
    }
%>