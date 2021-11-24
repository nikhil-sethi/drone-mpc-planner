import sqlite3
import datetime
import numbers
import pandas as pd
import dash
from dash import dash_table
from dash import dcc
from dash import html
import dash_bootstrap_components as dbc
from dash.dependencies import Output, Input, State
from aenum import Enum, NoAlias
from flask_login import current_user
from pats_c.lib.lib_patsc import open_systems_db


class main_buttons_options(Enum):
    _settings_ = NoAlias
    new_row = 'yes'
    edit = 'edit'
    delete = 'delete'
    undo = 'ok'
    save = 'ok'
    cancel = 'yes'

    def __str__(self):
        return f'{self.name}'


def get_available_tables():
    with open_systems_db() as con:
        sql_str = '''SELECT name FROM sqlite_master ORDER BY name COLLATE NOCASE'''
        tables = con.execute(sql_str).fetchall()
        tables = [table[0] for table in tables if 'sqlite' not in table[0] and 'connection' not in table[0] and 'change_log' not in table[0]]
    return tables


def get_column_info(table_name):
    columns = ['name', 'type', 'notnull', 'dft', 'unique', 'pk', 'fk']
    column_info = pd.DataFrame(columns=columns)
    with open_systems_db() as con:
        sql_str = f'''SELECT sql FROM sqlite_master WHERE name = '{table_name}' '''
        table_info = con.execute(sql_str).fetchone()
        if table_info:
            for column in table_info[0].split('\n'):
                new_entry = {}
                col_info = column.split('\t')
                if len(col_info) > 1 and not col_info[0]:
                    if 'KEY' not in col_info[1]:
                        new_entry['name'] = col_info[1]
                        if len(col_info) > 2:
                            new_entry['type'] = col_info[2].split(' ')[0].replace(',', '')
                            if 'NOT NULL' in col_info[2]:
                                new_entry['notnull'] = 1
                            if 'DEFAULT' in col_info[2]:
                                new_entry['dft'] = col_info[2].split('DEFAULT ')[1].replace(',', '').split()[0]
                            if 'UNIQUE' in col_info[2]:
                                new_entry['unique'] = 1
                        column_info = column_info.append(new_entry, ignore_index=True)
                    else:
                        if 'PRIMARY' in col_info[1]:
                            pk_cols = col_info[1].replace('(', ' ').replace(')', ' ').split()[2]
                            for col in pk_cols.split(','):
                                column_info.loc[column_info['name'] == col, 'pk'] = 1
                        elif 'FOREIGN' in col_info[1]:
                            fk_col = col_info[1].replace('(', ' ').replace(')', ' ').split()[2]
                            if 'REFERENCES ' in col_info[1]:
                                column_info.loc[column_info['name'] == fk_col, 'fk'] = col_info[1].replace(',', '').split('REFERENCES ')[1]
    return column_info.fillna(0)


def get_table_connection(table_name):
    connection_info = pd.DataFrame()
    with open_systems_db() as con:
        sql_str = f'''SELECT name FROM sqlite_master WHERE name LIKE '%{table_name[:-1] + '%connection'}' '''
        connection_name = con.execute(sql_str).fetchone()
        if connection_name:
            connection_info = get_column_info(connection_name[0])
            return connection_name[0], connection_info
    return '', connection_info


def get_fk_options(fk_str):
    if isinstance(fk_str, str):
        options = []
        with open_systems_db() as con:
            foreign_table = fk_str.split('"')[1]
            foreign_column = fk_str.split('"')[3]
            sql_str = f'''SELECT name,{foreign_column} FROM {foreign_table} ORDER BY name COLLATE NOCASE'''
            options = con.execute(sql_str).fetchall()
        return [{'label': name, 'value': id} for name, id in options]
    else:
        return []


def get_fk_values_for_ids(fk_str, id):
    with open_systems_db() as con:
        foreign_table = fk_str.split('"')[1]
        foreign_column = fk_str.split('"')[3]
        sql_str = f'''SELECT name,{foreign_column} FROM {foreign_table} WHERE {foreign_column} IN ({','.join([str(a) for a in id])}) '''
        values = con.execute(sql_str).fetchall()
    return values


def get_fk_ids_for_values(fk_str, values):
    with open_systems_db() as con:
        foreign_table = fk_str.split('"')[1]
        foreign_column = fk_str.split('"')[3]
        sql_str = f'''SELECT {foreign_column},name FROM {foreign_table} WHERE name IN ({','.join([f'"{str(a)}"' for a in values])}) '''
        ids = con.execute(sql_str).fetchall()
    return ids


def get_table_data(table_name):
    column_info = get_column_info(table_name)
    name_column_exsists = False
    sql_str = 'SELECT '
    join_str = ''
    for column in column_info.to_dict('records'):
        column_db_name = column['name'].replace('"', '')
        if column['fk']:
            fk_table = column['fk'].replace('"', '').split('(')[0]
            sql_str += f''' {fk_table}.name AS '{fk_table[:-1]}', '''
            join_str += f''' LEFT OUTER JOIN {fk_table} ON {fk_table}.{column_db_name} = {table_name}.{column_db_name} '''
        elif column_db_name != 'system':
            sql_str += f''' {table_name}.{column_db_name}, '''
        if column_db_name == 'name':
            name_column_exsists = True
    sql_str = sql_str[:-2] + f''' FROM {table_name} '''
    sql_str += join_str

    pk_name = column_info.loc[column_info['pk'] == 1, 'name'].values[0].replace('"', '')
    if name_column_exsists:
        sql_str += f''' ORDER BY {table_name}.name COLLATE NOCASE'''
    else:
        sql_str += f''' ORDER BY {table_name}.{pk_name} COLLATE NOCASE'''
    with open_systems_db() as con:
        data = pd.read_sql_query(sql_str, con)

    connection_name, connection_info = get_table_connection(table_name)
    if connection_name:
        for column in connection_info.to_dict('records'):
            if pk_name not in column['name']:
                fk_table = column['fk'].replace('"', '').split('(')[0]
                fk_column = column['fk'].replace('"', '').split('(')[1].replace(')', '')
                connection_str = f'''SELECT {fk_column} FROM {connection_name} WHERE {pk_name} = :id'''
                data[fk_table] = ''
                for id in data[pk_name]:
                    with open_systems_db() as con:
                        ids = con.execute(connection_str, {'id': id}).fetchall()
                        ids = [i[0] for i in ids]
                    fk_values = get_fk_values_for_ids(column['fk'], ids)
                    data.loc[data[pk_name] == id, fk_table] = ', '.join([value[0] for value in fk_values])

    return data


def get_max_id(table_name, column_info):
    pk_name = column_info.loc[column_info['pk'] == 1, 'name'].values[0].replace('"', '')
    with open_systems_db() as con:
        sql_str = f'''SELECT max({pk_name}) FROM {table_name}'''
        max_id = con.execute(sql_str).fetchone()[0]
    if not max_id:
        max_id = 0
    return max_id


def get_username():
    username = 'NOBODY'
    if current_user:
        if current_user.is_authenticated:
            username = current_user.username
    return username


def count_unsaved_changes():
    count = 0
    username = get_username()
    with open_systems_db() as con:
        sql_str = f'''SELECT COUNT(time) FROM change_log WHERE user = '{username}' AND saved = 0 '''
        count = con.execute(sql_str).fetchone()[0]
    return count


def add_row_to(table_name, column_info):
    err_msg = ''
    if table_name:
        max_id = get_max_id(table_name, column_info)
        new_column_input = []
        column_names = []
        for c in column_info.to_dict('records'):
            column_names.append(c['name'])
            if c['pk']:
                new_column_input.append(str(max_id + 1))
            elif c['name'] == '"system"':
                new_column_input.append(f'"pats{max_id + 1}"')
            elif c['dft']:
                new_column_input.append(str(c['dft']))
            elif c['unique']:
                new_column_input.append(f'"_Untitled{str(max_id+1)}"')
            elif c['notnull']:
                if c['type'] == 'TEXT':
                    new_column_input.append(str('""'))
                else:
                    new_column_input.append(str(0))
            else:
                new_column_input.append(str('NULL'))
        with open_systems_db() as con:
            sql_str = f'''INSERT INTO {table_name}({','.join(column_names)}) VALUES ({','.join(new_column_input)})'''
            try:
                con.execute(sql_str)
                con.commit()
            except sqlite3.Error as e:
                err_msg = 'DB error: ' + str(e)
                return False, err_msg

    return True, err_msg


def apply_unsaved_changes(data, table_name):
    sql_str = f'''SELECT table_name,column_name,id,value FROM change_log WHERE saved = 0 AND table_name LIKE '%{table_name[:-1] + '%'}' '''
    unsaved_changes = []
    with open_systems_db() as con:
        unsaved_changes = con.execute(sql_str).fetchall()
    if unsaved_changes:
        column_info = get_column_info(table_name)
        pks = column_info.loc[column_info['pk'] == 1, 'name'].values
        for table, column, index_list, value in unsaved_changes:
            index_list = [int(i) for i in index_list.split(',')]
            if 'connection' in table:
                connecting_tables = table.split('_')
                primairy_table_index = connecting_tables.index(table_name[:-1])
                second_table = connecting_tables[1 - primairy_table_index]  # The second table has index 1 of 0 and the primairy the other one and we remove the s of the first table
                value = value.split(',')[1 - primairy_table_index]
                table_values = data.loc[data[pks[0].replace('"', '')] == index_list[primairy_table_index], data.columns.str.contains(second_table)].values
                if table_values.size != 0:
                    table_values = table_values[0][0]  # this is because df uses a container list but only if the row exist, so first we check the row and then the value
                    if table_values:
                        table_values = table_values.split(', ')
                    else:
                        table_values = []
                    if column == 'ADD':
                        if value not in table_values:
                            table_values.append(value)
                    elif column == 'DELETE':
                        if value in table_values:
                            table_values.remove(value)
                    data.loc[data[pks[0].replace('"', '')] == index_list[primairy_table_index], data.columns.str.contains(second_table)] = ', '.join(sorted(table_values, key=str.lower))
            else:
                if len(pks) == 1:
                    if column_info.loc[column_info['name'] == f'"{column}"', 'fk'].values:
                        data.loc[data[pks[0].replace('"', '')] == index_list[0], column.replace('_id', '')] = get_fk_values_for_ids(column_info.loc[column_info['name'] == f'"{column}"', 'fk'].values[0], [value])[0][0]
                    elif column == 'DELETE':
                        data = data.drop(data[data[pks[0].replace('"', '')] == index_list[0]].index)
                    else:
                        data.loc[data[pks[0].replace('"', '')] == index_list[0], column] = value

    return data


def handle_new_row_button(choosen_table):
    unsaved_changes = count_unsaved_changes()
    if unsaved_changes:
        return True, [dbc.Label('You need to save before adding a new row.'), dbc.Label('Do you want to save right now?')]
    else:
        column_info = get_column_info(choosen_table)
        close, err_msg = add_row_to(choosen_table, column_info)
        return not close, [dbc.Label(err_msg)]  # close == Flase should open the modal so not close


def handle_edit_button(choosen_table, selected_rows, rows):
    children = []
    values = {}
    if len(selected_rows) == 1:
        values = rows[selected_rows[0]]
    column_info = get_column_info(choosen_table)
    for col in column_info.to_dict('records'):
        if not col['pk']:
            if not col['name'] == '"system"':
                value = ''
                if col['name'].replace('"', '') in values:
                    value = values[col['name'].replace('"', '')]
                if col['fk']:
                    label = ''
                    if col['fk'].split('"')[1][:-1] in values:  # To remove the s from customers, this should be solved better
                        label = values[col['fk'].split('"')[1][:-1]]
                    options = get_fk_options(col['fk'])
                    options_dict = {option['label']: option['value']for option in options}
                    children.extend([dbc.Label(col['name'].replace('"', '') + ':'), dcc.Dropdown(
                        id=col['name'],
                        options=options,
                        value='' if not label else options_dict[label],
                        style={'color': '#212121', },
                    )])
                elif not col['type'] == 'TEXT':
                    children.extend([dbc.Label(col['name'].replace('"', '') + ':'), dbc.Input(value=value, id=col['name'], type='number')])
                else:
                    if len(selected_rows) > 1:
                        value = '~'
                    children.extend([dbc.Label(col['name'].replace('"', '') + ':'), dbc.Input(value=value, id=col['name'], type='text')])
    connection_name, connection_info = get_table_connection(choosen_table)
    if not connection_info.empty:
        for col in connection_info.to_dict('records'):
            if choosen_table[:-1] not in col['name']:
                if col['fk']:
                    label = ''
                    if col['fk'].split('"')[1] in values:
                        label = values[col['fk'].split('"')[1]]
                    labels = label.split(', ')
                    options = get_fk_options(col['fk'])
                    options_dict = {option['label']: option['value']for option in options}
                    children.extend([dbc.Label(col['name'].replace('"', '') + ':'), dcc.Dropdown(
                        id=f"{connection_name}({col['name']})",
                        options=options,
                        value=[options_dict[label] for label in labels if label],
                        style={'color': '#212121', },
                        multi=True,
                    )])
    return children


def handle_delete_button(selected_rows):
    return html.Div(f'Are you sure you want to delete all {len(selected_rows)} rows?', style={'textAlign': 'center'})


def handle_undo_button():
    children = []
    username = get_username()
    with open_systems_db() as con:
        sql_last_change = f'''DELETE FROM change_log WHERE user='{username}' AND saved = 0 AND time = (SELECT MAX(time) FROM change_log WHERE user = '{username}' AND saved = 0 )'''
        try:
            con.execute(sql_last_change)
            con.commit()
        except Exception as e:
            err_msg = 'DB error: ' + str(e)
            children.append(html.Div(err_msg, style={'textAlign': 'center', 'color': 'red'}))
    return bool(children), children


def handle_save_button():
    err_msg = ''
    username = get_username()
    with open_systems_db() as con:
        sql_all_tables = f'''SELECT DISTINCT table_name FROM change_log WHERE user = '{username}' AND saved = 0 '''
        all_tables = con.execute(sql_all_tables).fetchall()
        tables_info = {}
        for table_name, in all_tables:
            tables_info[table_name] = get_column_info(table_name)

        sql_get_unsaved = f'''SELECT table_name,column_name,id,value FROM change_log WHERE user = '{username}' AND saved = 0 '''
        unsaved_changes = con.execute(sql_get_unsaved).fetchall()
        sql_save = ''' '''
        for table_name, column_name, index, value in unsaved_changes:
            id_list = [int(i) for i in index.split(',')]
            pks = tables_info[table_name].loc[tables_info[table_name]['pk'] == 1, 'name'].values
            if column_name == 'DELETE':
                sql_save += f'''DELETE FROM {table_name} WHERE { ' AND '.join([f'{pk} = {id}' for pk,id in zip(pks,id_list)]) } ; '''
            elif column_name == 'ADD':
                if 'connection' in table_name:
                    sql_save += f'''INSERT INTO {table_name}({','.join(pks)}) VALUES ({','.join([str(id) for id in id_list])}) ;'''
            elif len(pks) == 1:
                if value.isnumeric():
                    sql_save += f'''UPDATE {table_name} SET {column_name} = {value} WHERE { ' AND '.join([f'{pk} = {id}' for pk,id in zip(pks,id_list)]) } ; '''
                else:
                    sql_save += f'''UPDATE {table_name} SET {column_name} = '{value}' WHERE {' AND '.join([f'{pk} = {id}' for pk,id in zip(pks,id_list)])} ; '''

        if sql_save:
            sql_save += f'''UPDATE change_log SET saved = 1 WHERE user = '{username}' AND saved = 0 ;'''
            try:
                con.executescript(sql_save)
                con.commit()
            except Exception as e:
                err_msg = 'DB error: ' + str(e)
                return True, err_msg

    return False, ''


def handle_cancel_button():
    return html.Div('Are you sure you want to cancel all your unsaved changes?', style={'textAlign': 'center'})


def process_edit(children, choosen_table, selected_rows, current_data):
    column_info = get_column_info(choosen_table)
    err_msg = ''
    changes = []
    for child in children:
        if isinstance(child, dict):
            if 'props' in child:
                props = child['props']
                if 'value' in props:
                    value = props['value']
                    prop_type = 'dropdown'  # We only use dropdowns and input, only inputs got a type in prop, so if it doesn't it is a dropdown.
                    if 'type' in props:
                        prop_type = props['type']
                    column_name = props['id'].replace('"', '')
                    if prop_type == 'dropdown':
                        if len(selected_rows) > 1 and value:
                            changes.append({'column': column_name, 'value': value})
                        elif len(selected_rows) == 1:
                            changes.append({'column': column_name, 'value': value})
                    else:
                        if column_name in current_data.columns:
                            if isinstance(value, numbers.Number):
                                changes.append({'column': column_name, 'value': value})
                            elif len(selected_rows) > 1 and value != '~' and prop_type == 'text':
                                if column_info.loc[column_info['name'] == props['id'], 'unique'].values[0] == 1:
                                    err_msg = f'Can not change an unique value in column "{column_name}" of multiple rows.'
                                    return False, err_msg
                                else:
                                    changes.append({'column': column_name, 'value': value})
                            elif len(selected_rows) == 1 and value is not None:
                                if prop_type == 'text':
                                    if value != current_data[column_name].values[selected_rows[0]]:
                                        if column_info.loc[column_info['name'] == props['id'], 'unique'].values[0] == 1 and value in current_data[column_name].values:
                                            err_msg = f'Tried to edit unique column "{column_name}", this value is already used.'
                                            return False, err_msg
                                        else:
                                            changes.append({'column': column_name, 'value': value})
                                elif prop_type == 'number':
                                    if value != str(current_data[column_name].values[selected_rows[0]]):
                                        if column_info.loc[column_info['name'] == props['id'], 'unique'].values[0] == 1 and value in current_data[column_name].values:
                                            err_msg = f'Tried to edit unique column "{column_name}", this value is already used.'
                                            return False, err_msg
                                        else:
                                            changes.append({'column': column_name, 'value': value})

    sql_insert = '''INSERT INTO change_log(time,table_name,column_name,id,value,user,saved) VALUES '''
    now = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    username = get_username()
    table_info = get_column_info(choosen_table)
    pk = table_info.loc[table_info['pk'] == 1, 'name'].values[0].replace('"', '')
    for input in changes:
        for row in selected_rows:
            db_id = current_data.loc[row, pk]
            if input['column'] in current_data.columns:
                if input['value'] != current_data[input['column']].values[row]:
                    sql_insert += f'''({now},'{choosen_table}','{input['column']}',{db_id},'{input['value']}','{username}',0), '''
            elif 'id' in input['column'] and input['column'][:-3] in current_data.columns and input['value']:
                named_value = get_fk_values_for_ids(column_info.loc[column_info['name'] == f'"{input["column"]}"', 'fk'].values[0], [input['value']])[0][0]
                if named_value != current_data[input['column'][:-3]].values[row]:
                    sql_insert += f'''({now},'{choosen_table}','{input['column']}',{db_id},'{input['value']}','{username}',0), '''
            elif 'connection' in input['column']:
                connection_name, connection_info = get_table_connection(choosen_table)
                if connection_name in input['column']:
                    current_name = current_data.loc[row, 'name'] if 'name' in current_data else db_id
                    connecting_tables = connection_name.split('_')
                    primairy_table_index = connecting_tables.index(choosen_table[:-1])
                    edited_values = ['', '']
                    edited_values[primairy_table_index] = str(current_name)
                    edited_ids = ['', '']
                    edited_ids[primairy_table_index] = str(db_id)

                    con_col_name = input['column'].split('(')[-1].replace(')', '')
                    fk_str = connection_info.loc[connection_info['name'] == f'"{con_col_name}"', 'fk'].values[0]

                    table_values = current_data[fk_str.split('"')[1]].values[row].split(', ')
                    input_values = get_fk_values_for_ids(fk_str, input['value'])
                    for input_value, input_id in input_values:
                        if input_value not in table_values:
                            edited_values[1 - primairy_table_index] = str(input_value)
                            edited_ids[1 - primairy_table_index] = str(input_id)
                            sql_insert += f'''({now},'{connection_name}','ADD','{','.join(edited_ids)}','{','.join(edited_values)}','{username}',0), '''
                        else:
                            table_values.remove(input_value)
                    if len(selected_rows) == 1 and len(table_values):
                        table_ids = get_fk_ids_for_values(fk_str, table_values)
                        for table_id, table_name in table_ids:
                            edited_values[1 - primairy_table_index] = table_name
                            edited_ids[1 - primairy_table_index] = str(table_id)
                            sql_insert += f'''({now},'{connection_name}','DELETE','{','.join(edited_ids)}','{','.join(edited_values)}','{username}',0), '''

    if sql_insert.split(' ')[-2] != 'VALUES':
        with open_systems_db() as con:
            sql_insert = sql_insert[:-2] + ';'
            try:
                con.execute(sql_insert)
                con.commit()
                return True, err_msg
            except Exception as e:
                err_msg = 'DB error: ' + str(e)
    else:
        err_msg = 'You made no changes.'

    return False, err_msg


def process_delete(choosen_table, selected_rows, current_data):
    err_msg = ''
    sql_delete = '''INSERT INTO change_log(time,table_name,column_name,id,value,user,saved) VALUES '''
    now = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    username = get_username()
    table_info = get_column_info(choosen_table)
    pk = table_info.loc[table_info['pk'] == 1, 'name'].values[0].replace('"', '')

    connection_name, connection_info = get_table_connection(choosen_table)
    for row in selected_rows:
        db_id = current_data.loc[row, pk]
        current_name = current_data.loc[row, 'name'] if 'name' in current_data else db_id
        sql_delete += f'''({now},'{choosen_table}','DELETE',{db_id},'{current_name}','{username}',0), '''
        if connection_name:
            current_name = current_data.loc[row, 'name'] if 'name' in current_data else db_id
            connecting_tables = connection_name.split('_')
            primairy_table_index = connecting_tables.index(choosen_table[:-1])
            second_table = connecting_tables[1 - primairy_table_index]
            foreign_values = current_data.loc[row, current_data.columns.str.contains(second_table)][0]
            fk_str = connection_info.loc[connection_info['name'].str.contains(second_table), 'fk'].values[0]
            foreign_ids = get_fk_ids_for_values(fk_str, foreign_values.split(', '))

            ids = ['', '']
            ids[primairy_table_index] = str(db_id)
            values = ['', '']
            values[primairy_table_index] = str(current_name)

            for id, value in foreign_ids:
                ids[1 - primairy_table_index] = str(id)
                values[1 - primairy_table_index] = str(value)
                sql_delete += f'''({now},'{connection_name}','DELETE','{','.join(ids)}','{','.join(values)}','{username}',0), '''

    if sql_delete.split(' ')[-2] != 'VALUES':
        with open_systems_db() as con:
            sql_delete = sql_delete[:-2] + ';'
            try:
                con.execute(sql_delete)
                con.commit()
                return True, err_msg
            except Exception as e:
                err_msg = 'DB error: ' + str(e)
    else:
        err_msg = 'Nothing was deleted.'
    return False, err_msg


def process_cancel():
    username = get_username()
    with open_systems_db() as con:
        sql_str = f'''DELETE FROM change_log WHERE user='{username}' and saved = 0'''
        try:
            con.execute(sql_str)
            con.commit()
        except Exception as e:
            err_msg = 'DB error: ' + str(e)
            return False, err_msg
    return True, ''


def dash_application():
    print("Starting system-widget dash application!")
    app = dash.Dash(__name__, server=False, url_base_pathname='/sys-adm/', title='sys-adm')

    with open_systems_db() as con:
        table_exists = con.execute('''SELECT count(name) FROM sqlite_master WHERE type='table' AND name='change_log' ''').fetchone()[0]
        if not table_exists:
            sql_create = 'CREATE TABLE change_log(time INTEGER ,table_name TEXT,column_name TEXT,id TEXT,value TEXT,user TEXT,saved INTEGER)'
            con.execute(sql_create)
            con.commit()

    @app.callback(
        Output('table_holder', 'children'),
        Output('table_holder', 'style'),
        Input('table_dropdown', 'value'))
    def select_table(choosen_table):  # pylint: disable=unused-variable
        if not choosen_table:
            return dash_table.DataTable(), {'width': '75%', 'display': 'None', 'margin': 'auto'}

        data = get_table_data(choosen_table)

        data = apply_unsaved_changes(data, choosen_table)

        if isinstance(data, pd.DataFrame):
            columns = [{'name': column, 'id': column} for column in data.columns]
            data = data.to_dict('records')
            table = dash_table.DataTable(
                id='table',
                columns=columns,
                filter_action='native',
                sort_action='native',
                row_selectable='multi',
                data=data,
                style_table={'overflowX': 'auto', 'overflowY': 'auto'},
                # fixed_rows={'headers': True}, fixed rows gives bugs in dash-table 4.12.0 will be fixed in a later version plotly/dash-table#432
                css=[{'selector': 'table', 'rule': 'width: 100%;'}, {'selector': '.table, .dash-spreadsheet.dash-freeze-top, .dash-spreadsheet, .dash-virtualized, .dash-spreadsheet-inner', 'rule': 'max-height: calc(100vh - 250px);'}],
                style_cell={'backgroundColor': 'rgb(50, 50, 50)', 'color': 'white', 'padding': '5px', 'padding-right': '15px', 'textAlign': 'left'},
                style_filter={'backgroundColor': 'white', 'color': 'white'},
            )

            return table, {'width': '75%', 'display': 'block', 'margin': 'auto'}
        else:
            return dash_table.DataTable(), {'width': '75%', 'display': 'None', 'margin': 'auto'}

    @app.callback(
        Output('table_dropdown', 'disabled'), Output('dropdown_holder', 'title'),
        [Output('new_row_button', 'disabled'), Output('edit_button', 'disabled'), Output('delete_button', 'disabled'), Output('undo_button', 'disabled'), Output('save_button', 'disabled'), Output('cancel_button', 'disabled'), ],
        [Input('table_dropdown', 'value'), Input('table', 'derived_virtual_selected_rows')],
    )
    def activate_main_buttons(choosen_table, selected_rows):
        table_disable = False
        table_title = ''
        new_row_disable = True
        edit_disable = True
        delete_disable = True
        undo_disable = True
        save_disable = True
        cancel_disable = True
        if choosen_table:
            new_row_disable = False
            if selected_rows:
                edit_disable = False
                delete_disable = False
            if count_unsaved_changes():
                undo_disable = False
                save_disable = False
                cancel_disable = False
                table_disable = True
                table_title = 'Please save before selecting a different table.'

        return table_disable, table_title, new_row_disable, edit_disable, delete_disable, undo_disable, save_disable, cancel_disable

    @app.callback(
        Output('modal_trigger', 'disabled'),
        Output('modal_body', 'children'),
        Output('modal_button', 'children'),
        Output('modal_function_container', 'children'),
        [Input('new_row_button', 'n_clicks'), Input('edit_button', 'n_clicks'), Input('delete_button', 'n_clicks'), Input('undo_button', 'n_clicks'), Input('save_button', 'n_clicks'), Input('cancel_button', 'n_clicks'), ],
        [State('table_dropdown', 'value'), State('table', 'selected_rows'), State('table', 'data'), ],
    )
    def main_buttons(n_new, n_edit, n_delete, n_undo, n_save, n_cancel, choosen_table, selected_rows, rows):
        prop_id = dash.callback_context.triggered[0]['prop_id']
        children = []
        if 'new_row_button' in prop_id:
            show_modal, children = handle_new_row_button(choosen_table)
            return show_modal, children, main_buttons_options.new_row.value, str(main_buttons_options.new_row)
        elif 'edit_button' in prop_id:
            if selected_rows:
                children = handle_edit_button(choosen_table, selected_rows, rows)
                return True, children, main_buttons_options.edit.value, str(main_buttons_options.edit)
        elif 'delete_button' in prop_id:
            if selected_rows:
                children = handle_delete_button(selected_rows)
                return True, children, main_buttons_options.delete.value, str(main_buttons_options.delete)
        elif 'undo_button' in prop_id:
            show_modal, children = handle_undo_button()
            return show_modal, children, main_buttons_options.undo.value, str(main_buttons_options.undo)
        elif 'save_button' in prop_id:
            show_modal, err_msg = handle_save_button()
            children = html.Div(err_msg, style={'textAlign': 'center', 'color': 'red'})
            return show_modal, children, main_buttons_options.save.value, str(main_buttons_options.save)
        elif 'cancel_button' in prop_id:
            children = handle_cancel_button()
            return True, children, main_buttons_options.cancel.value, str(main_buttons_options.cancel)
        return False, children, 'button'

    @app.callback(
        Output("modal", "is_open"),
        Output('table_dropdown', 'value'),
        [Input('modal_trigger', 'disabled'), Input("modal_cancel", "n_clicks"), Input('modal_button', 'disabled')],
        [State("modal", "is_open"), State('table_dropdown', 'value'), ],
    )
    def toggle_modal(modal_trigger, n_modal_cancel, n_modal_button, is_open, choosen_table):
        prop_id = dash.callback_context.triggered[0]['prop_id']
        if "modal_cancel" in prop_id:
            return False, dash.no_update
        elif n_modal_button:
            return False, choosen_table
        elif modal_trigger:
            return True, dash.no_update
        return is_open, choosen_table

    @app.callback(
        Output('modal_button', 'disabled'),
        Output('modal_error', 'is_open'),
        Output('modal_error', 'children'),
        Input('modal_button', 'n_clicks'),
        Input('modal_body', 'children'),
        [State('table', 'data'), State('table', 'columns'), State('table_dropdown', 'value'), State('table', 'selected_rows'), State('modal_function_container', 'children'), ],
    )
    def close_modal(_, children, table_data, columns, choosen_table, selected_rows, modal_function):
        current_data = pd.DataFrame(table_data, columns=[column['name']for column in columns])
        err_msg = ''
        close = False
        if 'modal_button' in dash.callback_context.triggered[0]['prop_id']:
            if modal_function == main_buttons_options.edit.name:
                close, err_msg = process_edit(children, choosen_table, selected_rows, current_data)
            elif modal_function == main_buttons_options.cancel.name:
                close, err_msg = process_cancel()
            elif modal_function == main_buttons_options.delete.name:
                close, err_msg = process_delete(choosen_table, selected_rows, current_data)
            elif modal_function == main_buttons_options.new_row.name and count_unsaved_changes():
                show_modal, err_msg = handle_save_button()
                column_info = get_column_info(choosen_table)
                close, err_msg_add = add_row_to(choosen_table, column_info)
                err_msg += err_msg_add
                close = not show_modal and close
            else:
                close = True

        return close, bool(err_msg), err_msg

    def make_layout():
        if current_user:
            if current_user.is_authenticated:
                username = current_user.username.lower()
                if username == 'bram' or username == 'kevin' or username == 'sjoerd' or username == 'jorn':
                    tables = get_available_tables()
                    return html.Div(children=[
                        dbc.Navbar([
                            dbc.Col(html.H1(children='PATS-C admin')),
                            dbc.Col(html.H1()),
                            dbc.Col(
                                dbc.Nav(dbc.NavItem(dbc.NavLink("Sign out", href="/logout", external_link=True)), navbar=True),
                                width="auto",
                            )], color="#222", dark=True
                        ),
                        html.Div([
                            html.Div([
                                dcc.Dropdown(
                                    id='table_dropdown',
                                    options=[{'label': table, 'value': table} for table in tables],
                                    className='dash-bootstrap',
                                )
                            ], id='dropdown_holder', className='dash-bootstrap', style={'width': '40%', 'display': 'inline-block', 'float': 'left'}),
                            html.Div([
                                dbc.Button('edit', id='edit_button', color='dark', disabled=True, style={'width': '97%', 'margin-left': '3%'})
                            ], style={'width': '10%', 'display': 'inline-block'}),
                            html.Div([
                                dbc.Button('delete', id='delete_button', color='dark', disabled=True, style={'width': '97%', 'margin-left': '3%'})
                            ], style={'width': '10%', 'display': 'inline-block'}),
                            html.Div([
                                dbc.Button('new row', id='new_row_button', color='dark', disabled=True, style={'width': '97%', 'margin-left': '3%'})
                            ], style={'width': '10%', 'display': 'inline-block'}),
                            html.Div([
                                dbc.Button('undo', id='undo_button', color='dark', disabled=True, style={'width': '97%', 'margin-left': '3%'})
                            ], style={'width': '10%', 'display': 'inline-block'}),
                            html.Div([
                                dbc.Button('save', id='save_button', color='dark', disabled=True, style={'width': '97%', 'margin-left': '3%'})
                            ], style={'width': '10%', 'display': 'inline-block'}),
                            html.Div([
                                dbc.Button('cancel', id='cancel_button', color='dark', disabled=True, style={'width': '97%', 'margin-left': '3%'})
                            ], style={'width': '10%', 'display': 'inline-block', })
                        ], style={'display': 'block', 'textAlign': 'center', 'width': '70%', 'margin': 'auto'}),
                        html.Br(),
                        html.Div([
                            dash_table.DataTable(),
                        ], id='table_holder', className='dash-bootstrap', style={'width': '75%', 'display': 'block', 'margin': 'auto'}),
                        html.Div([dbc.Button(id='modal_trigger', style={'display': 'None'})]),
                        dbc.Modal(
                            [
                                html.Div([
                                    dbc.Alert(id='modal_error', color="danger", is_open=False, style={'margin': 'auto'})], style={'display': 'block', 'textAlign': 'center', 'width': '100%', 'margin': 'auto'}),
                                dbc.ModalBody([dbc.Label('', id='modal_label'),
                                              dbc.Input(id='modal_input', type="text")], id='modal_body'),
                                dbc.ModalFooter([
                                    dbc.Button('cancel', id='modal_cancel', color='danger', style={'width': '97%', 'margin-left': '3%'}),
                                    dbc.Button("save", id="modal_button", color="success", style={'width': '97%', 'margin-left': '3%'}),
                                ]),
                                html.Div([], id='modal_function_container', hidden=True)
                            ],
                            id="modal",
                            backdrop="static"
                        )
                    ])

                else:
                    print('Illegal user!')
        return html.Div([])

    app.layout = make_layout

    return app
