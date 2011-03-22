<%
            String benutzerName = (String) session.getAttribute("benutzer");
            if (benutzerName == null) {
                response.sendRedirect("login.jsp");
                //return;
            } else {
                response.sendRedirect("content.jsp");
            }

%>

<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//DE"
    "http://www.w3.org/TR/html4/loose.dtd">
<html>
    <head>
        <meta http-equiv="content-type"
              content="text/html; utf-8">
        <link rel="stylesheet" type="text/css"
              href="html.css" media="all">
    </head>
</html>