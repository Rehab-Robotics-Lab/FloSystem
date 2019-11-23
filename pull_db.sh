#!/bin/sh

set -e

scp nuc-admin@flo-nuc:/home/nuc-admin/db/flo.db ~/Downloads/
echo 'pulled the database into the local downloads folder'
