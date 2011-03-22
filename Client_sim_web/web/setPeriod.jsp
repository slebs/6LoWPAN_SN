<%@page import="java.util.GregorianCalendar"%>
<%@page import="java.util.Date"%>
<%@page import="jdbc.DBManager"%>
<%
    String benutzerName = (String) session.getAttribute("benutzer");

    if (benutzerName != null && request.getParameter("from") != null && request.getParameter("to") != null) {
        try {
            String pa = request.getParameter("from");
            System.out.println(pa);
            String[] array = pa.split("-");
            GregorianCalendar greg = new GregorianCalendar(Integer.parseInt(array[0]), Integer.parseInt(array[1]), Integer.parseInt(array[2]));
            Date from = greg.getTime();

            pa = request.getParameter("to");
            array = pa.split("-");
            greg = new GregorianCalendar(Integer.parseInt(array[0]), Integer.parseInt(array[1]), Integer.parseInt(array[2]));
            Date to = greg.getTime();

            session.setAttribute("from", from);
            session.setAttribute("to", to);
            System.out.println("setze Periode erfolgreich");
            response.sendRedirect("content.jsp");
        } catch (Exception ex) {

            response.sendRedirect("content.jsp");
        }
    } else {
        session.setAttribute("from", new Date(System.currentTimeMillis()));
        session.setAttribute("to", new Date(System.currentTimeMillis()));
        session.setAttribute("error", "2");
        System.out.println("SETIntervall fehler");
        out.print("nix geht");
    }
%>
