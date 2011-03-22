<%@page import="jdbc.DBManager"%>
<%
    String benutzerName = (String) session.getAttribute("benutzer");
    application.setAttribute("maxelements", request.getParameter("maxelements"));
    
    if (benutzerName != null && DBManager.getInstance().isAdmin(benutzerName)) {
        String pa = request.getParameter("intervall");
        int inter = Integer.parseInt(pa);
        if (DBManager.getInstance().setIntervall(inter, benutzerName)) {
            System.out.println("setIntervall erfolgreich");
            response.sendRedirect("content.jsp");
        } else {
            System.out.println("setIntervall fehlgeschlagen");

        }

    } else {
        session.setAttribute("error", "1");
        response.sendRedirect("login.jsp");
        System.out.println("SETIntervall fehler");
        out.print("nix geht");
    }
%>
