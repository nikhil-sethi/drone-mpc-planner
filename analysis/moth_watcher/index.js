var sqlite3 = require('sqlite3').verbose();
var d3 = require('d3');
let $ = require('jquery');

var margin = { top: 50, right: 0, bottom: 100, left: 30 },
    x_axis_width = 60
width = 1100 - margin.left - margin.right,
    gridSize = 40
legendElementWidth = gridSize * 2,
    buckets = 6,
    colors = ["#43AA8B", "#90BE6D", "#F9C74F", "#F8961E", "#F3722C", "#F94144"], // alternatively colorbrewer.YlGnBu[9]
    NAN_colour = "#909090"
times = ["0h", "1h", "2h", "3h", "4h", "5h", "6h", "7h", "8h", "9h", "10h",
    "11h", "12h", "13h", "14h", "15h", "16h", "17h", "18h",
    "19h", "20h", "21h", "22h", "23h"];
var selected_hours_1 = [];
var selected_hours_2 = [];
$('#analysebutt').attr('disabled', 'disabled');

var format_date_db = d3.time.format("%Y-%m-%dT%H:%M:%S+00:00").parse;
var format_date = d3.time.format("%d/%m/%Y");
var parse_date = d3.time.format("%d/%m/%Y").parse;

var div = d3.select("body").append("div")
    .attr("class", "tooltip-donut")
    .style("opacity", 0);

db_path = "./pats_records.db"
ds = getSystemsFromDb(db_path);
var parsed_dates = [];
ds.then(function (s) {

    d3.select("#selectButton")
        .selectAll('myOptions')
        .data(s)
        .enter()
        .append('option')
        .text(function (d) { return d; }) // text showed in the menu
        .attr("value", function (d) { return d; }) // corresponding value returned by the button

    d3.select("#selectButton").on("change", function (d) {
        selected_hours_1 = [];
        var selectedOption = d3.select(this).property("value")
        d3.select("svg").remove();
        parsed_dates = [];
        dates = getDatesOfSystemFromDb(db_path, selectedOption);
        dates.then(function (d) {
            d.forEach(function (d_) { parsed_dates.push(format_date_db(d_)) });
            data = getData(parsed_dates);
            heatmapChart(data, selectedOption);
        })
    })
    dates = getDatesOfSystemFromDb(db_path, s[0]);
    dates.then(function (d) {
        d.forEach(function (d_) { parsed_dates.push(format_date_db(d_)) });
        data = getData(parsed_dates);
        heatmapChart(data, s[0]);
    })
});

function getDatesRange(startDate, endDate) {
    var dates = []
    currentDate = startDate
    dates.push(startDate);
    addDays = function (days) {
        var date = new Date(this.valueOf());
        date.setDate(date.getDate() + days);
        return date;
    };
    while (currentDate <= endDate) {
        currentDate = addDays.call(currentDate, 1);
        dates.push(currentDate);
    }
    return dates;
};

function getData(dates) {
    data = [];
    date_range = getDatesRange(d3.min(dates), d3.max(dates)); //to make sure missing days are accounted for
    date_range.forEach(function (date) {

        for (var h = 0; h < 24; h++) {
            let current_date = new Date(date.getFullYear(), date.getMonth(), date.getDate(), h, 0, 0);
            let hour_later = new Date(current_date);
            hour_later.setHours(hour_later.getHours() + 1);
            filtered_dates = dates.filter((item) =>
                (item >= current_date) && (item < hour_later)
            );
            data.push({
                "day": format_date(current_date),
                "hour": h,
                "value": filtered_dates.length
            });
        }
    });
    return data;
}

function checkButton() {
    if (selected_hours_1.length + selected_hours_2.length != 0) {
        $('#analysebutt').removeAttr('disabled');
    } else {
        $('#analysebutt').attr('disabled', 'disabled');
    }
}

function heatmapChart(data, system) {

    var unique_dates = d3.map(data, function (d) { return d.day; }).keys()

    var svg = d3.select("#chart").append("svg")
        .attr("y", 200)
        .attr("width", width + x_axis_width + margin.left + margin.right)
        .attr("height", gridSize * unique_dates.length + margin.top + margin.bottom)
        .append("g")
        .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    var timeLabels = svg.selectAll(".timeLabel")
        .data(times)
        .enter().append("text")
        .text(function (d) { return d; })
        .attr("x", function (d, i) { return (i) * gridSize + x_axis_width + (gridSize / 2); })
        .attr("y", -8)
        .style("text-anchor", "middle")
        .attr("class", "timeLabel mono axis");

    var colorScale = d3.scale.threshold()
        .domain([1, 25, 50, 75, 100, 9001])
        .range(colors);


    svg.selectAll(".dayLabel")
        .data(unique_dates)
        .enter()
        .append("text")
        .text(function (d) { return d; })
        .attr("x", 50)
        .attr("y", function (d, i) { return i * gridSize + (gridSize / 2); })
        .attr("width", x_axis_width)
        .style("text-anchor", "end")
        .attr("class", "dayLabel mono axis axis-workweek")
        .on('click', function (d, i) {
            $(`[id*="${d}"]`).each(function (i, o) {
                if (selected_hours_1.includes(this.id)) {
                    this.style.width = "40";
                    this.style.height = "40";
                    this.style.stroke = "#E6E6E6";
                    selected_hours_1 = selected_hours_1.filter(item => item !== this.id)
                } else {
                    selected_hours_1.push(this.id);
                    this.style.width = "38";
                    this.style.height = "38";
                    this.style.stroke = "rgb(0,0,0)"
                }
                checkButton()
            })
        });


    var cards = svg.selectAll(".hour")
        .data(data, function (d) { return d.day + ':' + d.hour; });

    cards.append("title");

    cards.enter().append("rect")
        .attr("x", function (d, i) { return (d.hour * gridSize) + x_axis_width; })
        .attr("y", function (d, i) { return (Math.floor(i / 24)) * gridSize; })
        .attr("rx", 4)
        .attr("ry", 4)
        .attr("class", "hour bordered")
        .attr("id", function (d, i) { return d.day + "_" + d.hour; })
        .attr("width", gridSize)
        .attr("height", gridSize)
        .style("fill", colors[0])
        .on('mouseover', function (d, i) {
            d3.select(this).transition()
                .duration('50')
                .attr('opacity', '.85');
            div.transition()
                .duration(50)
                .style("opacity", 1);
            let num = d.value;
            div.html(num)
                .style("left", (d3.event.pageX + 10) + "px")
                .style("top", (d3.event.pageY - 15) + "px");
        })
        .on('mouseout', function (d, i) {
            d3.select(this).transition()
                .duration('50')
                .attr('opacity', '1');
            div.transition()
                .duration('50')
                .style("opacity", 0);
        })
        .on("contextmenu", function (d, i) {
            selection($(`[id*="${d.day + "_" + d.hour}"]`)[0], false);
            checkButton();
        });

    cards.transition().duration(100)
        .style("fill", function (d) {
            if ((d.hour > 8) && (d.hour <= 17)) { return "rgb(211,211,211)"; }
            if (d.value >= 0) { return colorScale(d.value); }
            else { return NAN_colour }
        });

    cards.select("title").text(function (d) { return d.value; });

    cards.exit().remove();

    $("rect").click(function () {
        selection(this);
        checkButton();
    });

    function selection(elem, left = true) {
        if (left) {
            if (selected_hours_1.includes(elem.id)) {
                elem.style.width = "40";
                elem.style.height = "40";
                elem.style.stroke = "rgb(255,255,255)";
                selected_hours_1 = selected_hours_1.filter(item => item !== elem.id)
            } else {
                if (selected_hours_2.includes(elem.id)) {
                    selected_hours_2 = selected_hours_2.filter(item => item !== elem.id)
                }
                selected_hours_1.push(elem.id);
                elem.style.width = "38";
                elem.style.height = "38";
                elem.style.stroke = "rgb(0,0,0)"
            }

        } else {
            if (selected_hours_2.includes(elem.id)) {
                elem.style.width = "40";
                elem.style.height = "40";
                elem.style.stroke = "rgb(255,255,255)";
                selected_hours_2 = selected_hours_2.filter(item => item !== elem.id)
            } else {
                if (selected_hours_1.includes(elem.id)) {
                    selected_hours_1 = selected_hours_1.filter(item => item !== elem.id)
                }
                selected_hours_2.push(elem.id);
                // elem.style.width = "38";
                // elem.style.height = "38";
                // elem.style.stroke = "rgb(102, 0, 255)"
                elem.style.fill = "rgb(211,211,211)"
            }
        }
    }

    $("#analysebutt").unbind().click(function () {
        localStorage.setItem("system", system);
        localStorage.setItem("selected_hours_1", selected_hours_1);
        localStorage.setItem("selected_hours_2", selected_hours_2);
        window.location = "analyses.html";
    });

    var legend = svg.selectAll(".legend")
        .data([0].concat([0.1, 25, 50, 75, 100]), function (d) { return d; });

    // .data([0].concat(colorScale.quantiles()), function (d) { return d; });

    legend.enter().append("g")
        .attr("class", "legend");

    legend.append("rect")
        .attr("x", function (d, i) { return legendElementWidth * i + x_axis_width; })
        .attr("y", gridSize * unique_dates.length + gridSize - 20)
        .attr("width", legendElementWidth)
        .attr("height", gridSize / 2)
        .style("fill", function (d, i) { return colors[i]; });

    legend.append("text")
        .attr("class", "mono")
        .text(function (d, i) {
            if (i == 0) {
                return "  " + Math.round(d);
            } else {
                return "> " + Math.round(d);
            }
        })
        .attr("x", function (d, i) { return legendElementWidth * i + x_axis_width; })
        .attr("y", gridSize * unique_dates.length + gridSize * 2 - 20);

    legend.exit().remove();
}

function getSystemsFromDb(database) {
    return new Promise((resolve, reject) => {
        const db = new sqlite3.Database(database);
        const systems = [];
        db.each("SELECT DISTINCT system from analytic_records", (err, row) => {
            if (err) {
                reject(err); // optional: you might choose to swallow errors.
            } else {
                systems.push(row.system); // accumulate the data
            }
        }, (err, n) => {
            if (err) {
                reject(err); // optional: again, you might choose to swallow this error.
            } else {
                resolve(systems); // resolve the promise
            }
        });
    });
}

function getDatesOfSystemFromDb(database, system) {
    return new Promise((resolve, reject) => {
        const db = new sqlite3.Database(database);
        const dates = [];
        db.each(`SELECT flight_time  from analytic_records where system = '${system}'`, (err, row) => {
            if (err) {
                reject(err); // optional: you might choose to swallow errors.
            } else {
                dates.push(row.flight_time); // accumulate the data
            }
        }, (err, n) => {
            if (err) {
                reject(err); // optional: again, you might choose to swallow this error.
            } else {
                resolve(dates); // resolve the promise
            }
        });
    });
}



