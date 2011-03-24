<%-- 
    Document   : content.jsp
    Created on : Dec 16, 2010, 3:49:29 PM
    Author     : simon
--%>
<%@page import="java.util.ArrayList"%>
<%@page import="serial.SerialComm"%>
<%@page import="java.util.Map.Entry"%>
<%@page import="jdbc.DBManager"%>
<%@page import="java.sql.Timestamp"%>

<%@page import="java.util.TreeMap"%>
<%
    int MAXNODES = 100;
    String benutzerName = (String) session.getAttribute("benutzer");
    if (benutzerName == null) {
        response.sendRedirect("login.jsp");
        session.setAttribute("error", "0");
        return;
    }
    // TreeMap tm = DBManager.getInstance().getDataMap();
    // int intervall = DBManager.getInstance().getIntervall();
%>
<%@page contentType="text/html" pageEncoding="UTF-8"%>
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//DE"
    "http://www.w3.org/TR/html4/loose.dtd">
<html>
    <head>
        <title>Messwerte</title>
        <!-- <meta http-equiv="refresh" content="100; URL=content.jsp">-->
        <link rel="stylesheet" type="text/css"
              href="html.css" media="all">
        <script type="text/javascript" src="js/jquery-1.4.4.min.js"></script>
        <script type="text/javascript" src="js/date.js"></script>
        <script type="text/javascript" src="js/jquery.datePicker.js"></script>


    </head>
    <body>
        <div id="wrappertop"></div>
        <div id="wrapper">

            <h1>SensorNetwork Test Control Interface</h1>
            <h2>Logged in as <%=benutzerName%></h2>

            <% if (DBManager.getInstance().isAdmin(benutzerName)) {%> 
            <p>User is admin</p>
            <%}%>

            <form action="sendCommands.jsp" method="get">
                <select name="port">

                    <%
                        ArrayList<String> ports = SerialComm.getInstance().getPortNames();

                        for (int i = 0; i < ports.size(); i++) {
                            out.println("<option value=\"" + ports.get(i) + "\">" + ports.get(i) + "</option>");
                        }
                    %><option value="/dev/ttyACM0">/dev/ttyACM0</option>
                </select>
                <select name="baud"> 
                    <option value="<% out.print(SerialComm.getInstance().getBaud());%>"><% out.print(SerialComm.getInstance().getBaud());%></option>
                    <option value="9600" >9600</option>
                </select>
                <input type="submit" value="Connect UART" name="connect"/>
            </form>

            <form action="sendCommands.jsp" method="get">
                <input type="submit" value="get connected Nodes address" name="getnodeaddress"/>
            </form>

            <form action="sendCommands.jsp" method="get">

                <select name="node"> 
                    <%for (int i = 1; i <= MAXNODES; i++) {
                            out.println("<option value=" + i + " >" + i + "</option>");
                        }%>

                </select>

                <input type="submit" value="Send getecho" name="getecho"/>

            </form>

            <form action="sendCommands.jsp" method="get">
                <select name="node"> 
                    <%for (int i = 1; i <= MAXNODES; i++) {
                            out.println("<option value=" + i + " >" + i + "</option>");
                        }%>
                </select>
                <input type="submit" value="Send getdata" name="getdata"/>
            </form>
            <form action="sendCommands.jsp" method="get">
                <input type="submit" value="Send getnodes" name="getnodes"/>
            </form>
            <form action="sendCommands.jsp" method="get">
                <input type="submit" value="Disconnect UART" name="disconnect"/>
            </form>

            <textarea rows="25" cols="80">
                <% out.println(SerialComm.getInstance().getConsoleOutput());%>
            </textarea>
            <form action="sendCommands.jsp" method="get">
                <input type="submit" value="clear" name="clear"/>
                <input type="submit" value="refresh" name="refresh" />
            </form>
            <form action="logout.jsp" method="get">
                <input type="submit" value="Logout" name="Logout" />
            </form>
        </div>
        <div id="wrapperbottom"></div>
    </body>
</html>
