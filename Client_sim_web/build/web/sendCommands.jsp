<%@page import="serial.SerialComm"%>
<%
    String benutzerName = (String) session.getAttribute("benutzer");
    if (benutzerName == null) {
        response.sendRedirect("login.jsp");
        session.setAttribute("error", "0");
        return;
    }

    SerialComm sc = SerialComm.getInstance();
    if (request.getParameter("getdata") != null) {

        SerialComm.getInstance().sendCommand(SerialComm.COMMAND_GETDATA, Integer.parseInt(request.getParameter("node")));
                Thread.sleep(150);
    } else if (request.getParameter("getecho") != null) {
        sc.sendCommand(SerialComm.COMMAND_GETECHO, Integer.parseInt(request.getParameter("node")));
        Thread.sleep(150);
    } else if (request.getParameter("getnodes") != null) {
        sc.sendCommand(SerialComm.COMMAND_GETNODES);
                Thread.sleep(150);
    } else if (request.getParameter("disconnect") != null) {
        sc.disconnect();
    } else if (request.getParameter("connect") != null) {
        sc.connect(request.getParameter("port"), Integer.parseInt(request.getParameter("baud")));
                Thread.sleep(150);
    } else if (request.getParameter("getnodeaddress") != null) {
        sc.sendCommand(SerialComm.COMMAND_GETCONNECTEDNODE);
                Thread.sleep(150);
    } else if (request.getParameter("clear")!=null) {
        sc.setDataString("");
    }
    response.sendRedirect("content.jsp");
%>